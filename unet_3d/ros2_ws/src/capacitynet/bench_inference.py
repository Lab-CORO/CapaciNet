#!/usr/bin/env python3
"""Export the new checkpoint to ONNX/TensorRT and benchmark inference latency
on this machine: PyTorch FP32, PyTorch FP16, TensorRT FP16, TensorRT INT8.

Model : /home/ros2_ws/src/best_checkpoint.pytorch
Config: /home/ros2_ws/src/test.yaml  (predict-style config, model + model_path)
"""
import argparse
import os
import statistics
import sys
import time

import torch
import yaml

sys.path.insert(0, '/opt/conda/envs/ros2_torch/lib/python3.10/site-packages')
sys.path.insert(0, '/home/ros2_ws/src/capacitynet/capacitynet/config')  # for trt_model.py

from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils

CONFIG_PATH = "/home/ros2_ws/src/test.yaml"
ONNX_PATH   = "/home/ros2_ws/src/capacitynet/config/unet3d.onnx"
ENGINE_PATH = "/home/ros2_ws/src/capacitynet/config/unet3d.trt"

# INT8 reuses artifacts already produced by scripts/export_to_trt.py / calibrate_int8.py
# for this same checkpoint — no export or calibration is run from this script.
ONNX_FP32_PATH    = "/home/ros2_ws/src/capacitynet/config/unet3d.onnx"
CALIB_CACHE_PATH  = "/home/ros2_ws/src/capacitynet/config/int8_calibration.cache"
ENGINE_INT8_PATH  = "/home/ros2_ws/src/capacitynet/config/unet3d_int8.trt"

SPATIAL = 128
N_WARMUP = 10
N_RUNS = 50


def load_config():
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f)


def load_model_fp32(config):
    model = get_model(config['model'])
    utils.load_checkpoint(config['model_path'], model)
    model.eval()
    model.to('cuda')
    return model


def export_onnx(model, onnx_path, fp16=False):
    import onnx
    dtype = torch.float16 if fp16 else torch.float32
    print(f"[ONNX] Exporting spatial={SPATIAL}^3 dynamic batch, dtype={dtype} -> {onnx_path}")
    model = model.to(dtype)
    dummy = torch.zeros(1, 1, SPATIAL, SPATIAL, SPATIAL, dtype=dtype, device='cuda')
    torch.onnx.export(
        model, dummy, onnx_path,
        opset_version=17,
        input_names=['input'], output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}},
        do_constant_folding=True,
    )
    onnx.checker.check_model(onnx.load(onnx_path))
    print(f"[ONNX] OK  ({os.path.getsize(onnx_path)/1e6:.1f} MB)")


def build_trt_engine(onnx_path, engine_path, spatial=SPATIAL, max_batch=1):
    """TRT 11 dropped the weakly-typed BuilderFlag.FP16 global switch — precision
    now follows the ONNX graph's own tensor dtypes ('strongly typed' networks).
    The ONNX file must already be exported in FP16 (see export_onnx(fp16=True))."""
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
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

    print(f"[TRT] Building FP16 engine (spatial={spatial}^3, max_batch={max_batch})... this can take several minutes")
    t0 = time.time()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError('TRT engine build failed')
    with open(engine_path, 'wb') as f:
        f.write(serialized)
    print(f"[TRT] Built in {time.time()-t0:.1f}s -> {engine_path}  ({os.path.getsize(engine_path)/1e6:.1f} MB)")


def build_trt_engine_int8(onnx_path, engine_path, cache_path, spatial=SPATIAL, max_batch=1):
    """Build an INT8 engine from an existing calibration cache — no calibration data
    is loaded and no batches are scanned here. TensorRT applies the cached per-tensor
    scales as soon as read_calibration_cache() returns them, without ever calling
    get_batch(); if the cache doesn't cover every tensor the build fails loudly instead
    of silently recalibrating.

    Implicit (calibrator-based) INT8 only works on a weakly-typed network, so unlike
    build_trt_engine() above this must NOT set STRONGLY_TYPED.
    """
    import tensorrt as trt

    if not os.path.exists(cache_path):
        raise RuntimeError(
            f'No INT8 calibration cache at {cache_path}. '
            'Run scripts/calibrate_int8.py once to generate it.')

    class _CacheOnlyCalibrator(trt.IInt8EntropyCalibrator2):
        def get_batch_size(self):
            return 1

        def get_batch(self, names):
            return None

        def read_calibration_cache(self):
            with open(cache_path, 'rb') as f:
                return f.read()

        def write_calibration_cache(self, cache):
            pass

    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, "")

    builder = trt.Builder(logger)
    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(network_flags)
    parser = trt.OnnxParser(network, logger)

    with open(onnx_path, 'rb') as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(parser.get_error(i))
            raise RuntimeError('ONNX parse failed')

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 4 << 30)
    config.set_flag(trt.BuilderFlag.INT8)
    config.set_flag(trt.BuilderFlag.FP16)  # fallback for layers INT8 can't handle
    config.int8_calibrator = _CacheOnlyCalibrator()

    profile = builder.create_optimization_profile()
    shape_min = (1, 1, spatial, spatial, spatial)
    shape_opt = (max_batch, 1, spatial, spatial, spatial)
    profile.set_shape('input', shape_min, shape_opt, shape_opt)
    config.add_optimization_profile(profile)

    print(f"[TRT] Building INT8 engine from cached scales (spatial={spatial}^3, max_batch={max_batch})...")
    t0 = time.time()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError('TRT INT8 engine build failed')
    with open(engine_path, 'wb') as f:
        f.write(serialized)
    print(f"[TRT] Built in {time.time()-t0:.1f}s -> {engine_path}  ({os.path.getsize(engine_path)/1e6:.1f} MB)")


def bench(fn, x, n_warmup=N_WARMUP, n_runs=N_RUNS):
    for _ in range(n_warmup):
        fn(x)
    torch.cuda.synchronize()
    times = []
    for _ in range(n_runs):
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        fn(x)
        torch.cuda.synchronize()
        times.append((time.perf_counter() - t0) * 1000.0)
    return times


def report(label, times):
    print(f"\n=== {label} ===")
    print(f"  n runs      : {len(times)}")
    print(f"  mean        : {statistics.mean(times):.2f} ms")
    print(f"  median      : {statistics.median(times):.2f} ms")
    print(f"  min / max   : {min(times):.2f} / {max(times):.2f} ms")
    print(f"  stdev       : {statistics.stdev(times):.2f} ms")
    print(f"  throughput  : {1000.0/statistics.median(times):.2f} volumes/s")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--skip-export', action='store_true')
    p.add_argument('--skip-build', action='store_true')
    args = p.parse_args()

    config = load_config()
    print(f"GPU: {torch.cuda.get_device_name(0)}")

    model_fp32 = load_model_fp32(config)
    x = torch.zeros(1, 1, SPATIAL, SPATIAL, SPATIAL, dtype=torch.float32, device='cuda')

    results = {}

    # --- PyTorch FP32 ---
    torch.backends.cudnn.benchmark = True
    with torch.no_grad():
        t = bench(lambda inp: model_fp32(inp), x)
    results['PyTorch FP32'] = t
    report('PyTorch FP32', t)

    # --- PyTorch FP16 ---
    model_fp16 = load_model_fp32(config).half()
    x_fp16 = x.half()
    with torch.no_grad():
        t = bench(lambda inp: model_fp16(inp), x_fp16)
    results['PyTorch FP16'] = t
    report('PyTorch FP16', t)

    # --- TensorRT FP16 ---
    if not args.skip_export:
        export_onnx(model_fp32, ONNX_PATH, fp16=True)
    if not args.skip_build:
        build_trt_engine(ONNX_PATH, ENGINE_PATH)

    from capacitynet.rm_model.trt_model import TRTModel
    trt_model = TRTModel(ENGINE_PATH)
    trt_model.warmup(spatial=SPATIAL)
    torch.cuda.synchronize()

    x_cuda = x.clone()
    t = bench(lambda inp: trt_model.infer(inp), x_cuda)
    results['TensorRT FP16'] = t
    report('TensorRT FP16', t)

    # --- TensorRT INT8 (reuses the existing FP32 ONNX + calibration cache from
    # scripts/export_to_trt.py / calibrate_int8.py — no export or calibration here) ---
    if not args.skip_build:
        build_trt_engine_int8(ONNX_FP32_PATH, ENGINE_INT8_PATH, CALIB_CACHE_PATH)

    trt_model_int8 = TRTModel(ENGINE_INT8_PATH)
    trt_model_int8.warmup(spatial=SPATIAL)
    torch.cuda.synchronize()

    t = bench(lambda inp: trt_model_int8.infer(inp), x_cuda)
    results['TensorRT INT8'] = t
    report('TensorRT INT8', t)

    print("\n\n=== SUMMARY (median latency, batch=1, 128^3) ===")
    for label, t in results.items():
        print(f"  {label:<16}: {statistics.median(t):8.2f} ms  ({1000.0/statistics.median(t):6.2f} vol/s)")


if __name__ == '__main__':
    main()
