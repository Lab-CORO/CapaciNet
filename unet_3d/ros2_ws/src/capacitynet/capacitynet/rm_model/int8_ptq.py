#!/usr/bin/env python3
"""PTQ INT8 quantization of the UNet3D checkpoint via nvidia-modelopt.

TensorRT 11's Python API dropped the old implicit-calibration INT8 workflow
(IInt8EntropyCalibrator2 etc. no longer exist), so capacitynet's own
scripts/calibrate_int8.py cannot run here. This script uses the modern
explicit-quantization path instead:

  1. Export the FP32 checkpoint to ONNX (dynamic batch).
  2. modelopt.onnx.quantization.quantize() calibrates entropy-based INT8
     scales from real calibration volumes and inserts QuantizeLinear /
     DequantizeLinear (Q/DQ) nodes into the ONNX graph.
  3. Build a TensorRT engine (STRONGLY_TYPED network — TRT 11 has no
     BuilderFlag.INT8 either; precision now follows the Q/DQ-annotated graph).
"""
import argparse
import glob
import os
import random
import sys
import time

import h5py
import numpy as np
import torch
import yaml

sys.path.insert(0, '/home/ros2_ws/src/capacitynet/capacitynet')  # for trt_model.py
from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils

CONFIG_PATH = "/home/ros2_ws/src/test.yaml"
CALIB_DIR = "/home/ros2_ws/src/dataset_validation"
ONNX_FP32_PATH = "/home/ros2_ws/src/unet3d_fp32.onnx"
ONNX_INT8_PATH = "/home/ros2_ws/src/unet3d_int8.onnx"
ENGINE_INT8_PATH = "/home/ros2_ws/src/unet3d_int8.trt"
SPATIAL = 128


def normalize(arr):
    """Match pytorch3dunet Normalize (min-max to [-1, 1]) used at training time."""
    mn, mx = float(arr.min()), float(arr.max())
    norm01 = (arr - mn) / (mx - mn + 1e-10)
    return np.clip(2.0 * norm01 - 1.0, -1.0, 1.0).astype(np.float32)


def load_calib_stack(n_samples, seed=0):
    files = sorted(glob.glob(os.path.join(CALIB_DIR, '*.h5')))
    rng = random.Random(seed)
    rng.shuffle(files)
    files = files[:n_samples]
    print(f"[calib] using {len(files)} files from {CALIB_DIR}")
    vols = []
    for fp in files:
        with h5py.File(fp, 'r') as f:
            raw = f['raw'][:].astype(np.float32)
        vols.append(normalize(raw)[None])  # (1, D, H, W)
    stack = np.stack(vols, axis=0)  # (N, 1, D, H, W)
    return stack.astype(np.float32)


def export_onnx_fp32(config, onnx_path):
    model = get_model(config['model'])
    utils.load_checkpoint(config['model_path'], model)
    model.eval().to('cuda')
    dummy = torch.zeros(1, 1, SPATIAL, SPATIAL, SPATIAL, dtype=torch.float32, device='cuda')
    torch.onnx.export(
        model, dummy, onnx_path,
        opset_version=17,
        input_names=['input'], output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}},
        do_constant_folding=True,
    )
    print(f"[ONNX fp32] {onnx_path}  ({os.path.getsize(onnx_path)/1e6:.1f} MB)")
    return model


def quantize_onnx(onnx_path, out_path, calib_stack):
    import modelopt.onnx.quantization as moq
    print(f"[PTQ] entropy calibration on {calib_stack.shape[0]} volumes ({calib_stack.nbytes/1e9:.2f} GB)...")
    t0 = time.time()
    moq.quantize(
        onnx_path,
        quantize_mode='int8',
        calibration_data={'input': calib_stack},
        calibration_method='entropy',
        output_path=out_path,
        high_precision_dtype='fp16',
    )
    print(f"[PTQ] done in {time.time()-t0:.1f}s -> {out_path}  ({os.path.getsize(out_path)/1e6:.1f} MB)")


def build_trt_engine(onnx_path, engine_path, spatial=SPATIAL, max_batch=1):
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.INFO)
    trt.init_libnvinfer_plugins(logger, "")

    builder = trt.Builder(logger)
    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.STRONGLY_TYPED)
    network = builder.create_network(network_flags)
    parser = trt.OnnxParser(network, logger)

    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(parser.get_error(i))
            raise RuntimeError('ONNX parse failed')

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 4 << 30)

    profile = builder.create_optimization_profile()
    shape_min = (1, 1, spatial, spatial, spatial)
    shape_opt = (max_batch, 1, spatial, spatial, spatial)
    profile.set_shape('input', shape_min, shape_opt, shape_opt)
    config.add_optimization_profile(profile)

    print(f"[TRT] Building INT8 (Q/DQ) engine (spatial={spatial}^3)...")
    t0 = time.time()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError('TRT engine build failed')
    with open(engine_path, 'wb') as f:
        f.write(serialized)
    print(f"[TRT] Built in {time.time()-t0:.1f}s -> {engine_path}  ({os.path.getsize(engine_path)/1e6:.1f} MB)")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--n-calib', type=int, default=128)
    p.add_argument('--skip-export', action='store_true')
    p.add_argument('--skip-quantize', action='store_true')
    p.add_argument('--skip-build', action='store_true')
    args = p.parse_args()

    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f)

    if not args.skip_export:
        export_onnx_fp32(config, ONNX_FP32_PATH)

    if not args.skip_quantize:
        calib_stack = load_calib_stack(args.n_calib)
        quantize_onnx(ONNX_FP32_PATH, ONNX_INT8_PATH, calib_stack)

    if not args.skip_build:
        build_trt_engine(ONNX_INT8_PATH, ENGINE_INT8_PATH)

    print("\nDone.")
    print(f"  ONNX fp32 : {ONNX_FP32_PATH}")
    print(f"  ONNX int8 : {ONNX_INT8_PATH}")
    print(f"  Engine    : {ENGINE_INT8_PATH}")


if __name__ == '__main__':
    main()
