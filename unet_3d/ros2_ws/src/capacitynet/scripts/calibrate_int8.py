#!/usr/bin/env python3
"""Build a TensorRT INT8 engine for UNet3D with entropy calibration.

Usage:
    python3 scripts/calibrate_int8.py [--data-dir config/calibration_data] \
                                       [--onnx config/unet3d.onnx] \
                                       [--engine config/unet3d_int8.trt] \
                                       [--cache config/int8_calibration.cache] \
                                       [--spatial 128] [--batch 1]

Place 100-500 training H5 files (with a 'raw' dataset, shape 128³) in the
calibration_data directory, then run this script once. The resulting engine
can be activated by pointing trt_engine_path in config/test_reach.yaml to
config/unet3d_int8.trt.

The calibration cache is saved separately so you can rebuild the engine
without re-running calibration (just pass --cache to reuse it).

Build time: ~30-60 min on Jetson Orin AGX.
"""

import argparse
import glob
import math
import os
import sys

import h5py
import numpy as np
import torch
import tensorrt as trt

sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')

CONFIG_DIR = '/home/ros2_ws/src/capacitynet/config'
DEFAULT_DATA_DIR = os.path.join(CONFIG_DIR, 'calibration_data')
DEFAULT_ONNX    = os.path.join(CONFIG_DIR, 'unet3d.onnx')
DEFAULT_ENGINE  = os.path.join(CONFIG_DIR, 'unet3d_int8.trt')
DEFAULT_CACHE   = os.path.join(CONFIG_DIR, 'int8_calibration.cache')
DEFAULT_SPATIAL = 128

_logger = trt.Logger(trt.Logger.INFO)
trt.init_libnvinfer_plugins(_logger, '')


# ---------------------------------------------------------------------------
# Calibration data loader
# ---------------------------------------------------------------------------

def _load_h5_files(data_dir: str) -> list:
    """Collect all H5 files from data_dir."""
    patterns = ['*.h5', '*.hdf5', '*.hdf', '*.hd5']
    files = []
    for p in patterns:
        files.extend(glob.glob(os.path.join(data_dir, p)))
    return sorted(files)


def _preprocess(h5_path: str, spatial: int) -> torch.Tensor:
    """Load and preprocess one H5 file into a (1,1,D,H,W) float32 CUDA tensor.

    Applies the same pipeline as the ROS2 inference node:
      1. Read 'raw' field (occupancy: 0=free, 1=occupied)
      2. Invert: 1 - x  → 1=free, 0=occupied
      3. Cast to float32 and reshape to (1, 1, spatial, spatial, spatial)
    """
    with h5py.File(h5_path, 'r') as f:
        raw = np.array(f['raw'], dtype=np.float32)

    # Crop or resize to target spatial size if needed
    if raw.shape != (spatial, spatial, spatial):
        t = torch.from_numpy(raw).unsqueeze(0).unsqueeze(0)  # (1,1,D,H,W)
        t = torch.nn.functional.interpolate(t, size=(spatial, spatial, spatial),
                                             mode='nearest')
        raw = t.squeeze().numpy()

    inverted = (1.0 - raw).reshape(1, 1, spatial, spatial, spatial)
    return torch.from_numpy(inverted)


# ---------------------------------------------------------------------------
# TRT INT8 calibrator
# ---------------------------------------------------------------------------

class H5Int8Calibrator(trt.IInt8EntropyCalibrator2):
    """Feeds voxel grids from H5 training files to the TRT calibrator.

    Uses IInt8EntropyCalibrator2 (recommended for CNNs).
    Saves/loads a calibration cache so the dataset scan can be skipped on
    subsequent engine rebuilds.
    """

    def __init__(self, h5_files: list, cache_path: str, spatial: int = 128):
        super().__init__()
        self._cache_path = cache_path
        self._spatial = spatial
        self._idx = 0

        print(f'[Calibrator] Loading {len(h5_files)} H5 files...')
        self._batches = []
        for i, path in enumerate(h5_files):
            try:
                t = _preprocess(path, spatial)  # (1,1,D,H,W) float32 CPU
                self._batches.append(t)
                if (i + 1) % 50 == 0:
                    print(f'    {i+1}/{len(h5_files)} loaded')
            except Exception as e:
                print(f'    [WARN] Skipping {path}: {e}')
        print(f'[Calibrator] {len(self._batches)} samples ready.')

        # Pre-allocate GPU buffer (fixed address baked into calibration).
        self._gpu_buf = torch.zeros(1, 1, spatial, spatial, spatial,
                                    dtype=torch.float32, device='cuda')

    def get_batch_size(self) -> int:
        return 1

    def get_batch(self, names):
        if self._idx >= len(self._batches):
            return None
        self._gpu_buf.copy_(self._batches[self._idx])
        self._idx += 1
        if self._idx % 50 == 0:
            print(f'    Calibrating... {self._idx}/{len(self._batches)}')
        return [self._gpu_buf.data_ptr()]

    def read_calibration_cache(self):
        if os.path.exists(self._cache_path):
            print(f'[Calibrator] Reusing existing cache: {self._cache_path}')
            with open(self._cache_path, 'rb') as f:
                return f.read()
        return None

    def write_calibration_cache(self, cache):
        with open(self._cache_path, 'wb') as f:
            f.write(cache)
        print(f'[Calibrator] Cache written: {self._cache_path}')


# ---------------------------------------------------------------------------
# Engine builder
# ---------------------------------------------------------------------------

def build_int8_engine(onnx_path: str, engine_path: str, cache_path: str,
                       h5_files: list, spatial: int):
    builder = trt.Builder(_logger)
    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(network_flags)
    parser = trt.OnnxParser(network, _logger)

    print(f'[1/3] Parsing ONNX: {onnx_path}')
    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(f'    ONNX parse error: {parser.get_error(i)}')
            sys.exit(1)
    print('    ONNX parsed OK')

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 16384 * 1024 * 1024)
    config.set_flag(trt.BuilderFlag.INT8)
    config.set_flag(trt.BuilderFlag.FP16)   # FP16 fallback for layers not supporting INT8
    config.clear_flag(trt.BuilderFlag.TF32)

    # Optimization profile — same shape constraints as the FP16 engine.
    max_batch = math.floor((2**31 - 1) / (96 * spatial**3))
    max_batch = max(1, max_batch)
    profile = builder.create_optimization_profile()
    shape_min = (1, 1, spatial, spatial, spatial)
    shape_max = (max_batch, 1, spatial, spatial, spatial)
    profile.set_shape('input', shape_min, shape_min, shape_max)
    config.add_optimization_profile(profile)

    print(f'[2/3] Running INT8 calibration ({len(h5_files)} samples)...')
    calibrator = H5Int8Calibrator(h5_files, cache_path, spatial)
    config.int8_calibrator = calibrator

    print(f'[3/3] Building INT8 engine (this may take 30-60 min on Jetson Orin)...')
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        print('ERROR: Engine build failed.')
        sys.exit(1)

    with open(engine_path, 'wb') as f:
        f.write(serialized)

    size_mb = os.path.getsize(engine_path) / 1e6
    print(f'\nDone. Engine saved: {engine_path}  ({size_mb:.1f} MB)')
    print(f'\nTo activate, set in config/test_reach.yaml:')
    print(f'  trt_engine_path: "{engine_path}"')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description='Build TRT INT8 engine with H5 calibration data')
    p.add_argument('--data-dir', default=DEFAULT_DATA_DIR,
                   help='Directory containing calibration H5 files (key: raw)')
    p.add_argument('--onnx',    default=DEFAULT_ONNX)
    p.add_argument('--engine',  default=DEFAULT_ENGINE)
    p.add_argument('--cache',   default=DEFAULT_CACHE,
                   help='Calibration cache file (reused if it already exists)')
    p.add_argument('--spatial', type=int, default=DEFAULT_SPATIAL,
                   help='Spatial dimension (cubic) — must match the ONNX export')
    return p.parse_args()


def main():
    args = parse_args()

    print('=== UNet3D INT8 TensorRT Calibration & Build ===')
    print(f'Data dir : {args.data_dir}')
    print(f'ONNX     : {args.onnx}')
    print(f'Engine   : {args.engine}')
    print(f'Cache    : {args.cache}')
    print(f'Spatial  : {args.spatial}³')
    print()

    if not os.path.isfile(args.onnx):
        print(f'ERROR: ONNX file not found: {args.onnx}')
        print('Run scripts/export_to_trt.py first to generate the ONNX model.')
        sys.exit(1)

    h5_files = _load_h5_files(args.data_dir)
    if not h5_files and not os.path.exists(args.cache):
        print(f'ERROR: No H5 files found in {args.data_dir} and no existing cache.')
        print(f'Place training H5 files (key: raw, shape: {args.spatial}³) in that folder.')
        sys.exit(1)

    if h5_files:
        print(f'Found {len(h5_files)} calibration files.')
    else:
        print(f'No H5 files found — will reuse existing cache: {args.cache}')

    build_int8_engine(args.onnx, args.engine, args.cache, h5_files, args.spatial)


if __name__ == '__main__':
    main()
