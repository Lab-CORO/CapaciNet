#!/usr/bin/env python3
"""One-shot conversion script: PyTorch checkpoint -> ONNX -> TensorRT engine.

Usage:
    python3 scripts/export_to_trt.py [--spatial 152] [--no-trtexec]

Prerequisites:
    pip install onnx
    TensorRT installed at /usr/src/tensorrt/bin/trtexec

Constraint: spatial dims are baked at 152³ in the ONNX graph (skip connections
use concrete sizes from encoder features during tracing). Re-export if grid size changes.
"""

import argparse
import subprocess
import sys
import os

import torch
import yaml
import onnx

# Make sure pytorch3dunet is importable from the ROS install
sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')

from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils

TRTEXEC = '/usr/src/tensorrt/bin/trtexec'
CONFIG_DIR = '/home/ros2_ws/src/capacitynet/config'
DEFAULT_CONFIG = os.path.join(CONFIG_DIR, 'test_reach.yaml')
DEFAULT_ONNX = os.path.join(CONFIG_DIR, 'unet3d.onnx')
DEFAULT_ENGINE = os.path.join(CONFIG_DIR, 'unet3d.trt')
DEFAULT_LOG = os.path.join(CONFIG_DIR, 'trtexec_build.log')


def parse_args():
    p = argparse.ArgumentParser(description='Export UNet3D to TensorRT')
    p.add_argument('--config', default=DEFAULT_CONFIG)
    p.add_argument('--onnx', default=DEFAULT_ONNX)
    p.add_argument('--engine', default=DEFAULT_ENGINE)
    p.add_argument('--spatial', type=int, default=152,
                   help='Spatial dimension (cubic). Must match deployment voxel grid size.')
    p.add_argument('--no-trtexec', action='store_true',
                   help='Only export ONNX, skip TensorRT compilation')
    return p.parse_args()


def load_model_fp32(config_path):
    config = yaml.safe_load(open(config_path, 'r'))
    model = get_model(config['model'])
    utils.load_checkpoint(config['model_path'], model)
    model.eval()
    # Export from FP32 — TRT decides per-layer precision with --fp16.
    # Do NOT call .half() here: it bakes FP16 weights into ONNX and reduces TRT's freedom.
    return model, config


def export_onnx(model, spatial, onnx_path):
    print(f'[1/3] Exporting ONNX  (spatial={spatial}³, batch=dynamic)...')
    dummy = torch.zeros(1, 1, spatial, spatial, spatial, dtype=torch.float32)

    torch.onnx.export(
        model,
        dummy,
        onnx_path,
        opset_version=17,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input':  {0: 'batch_size'},
            'output': {0: 'batch_size'},
        },
        do_constant_folding=True,
    )
    print(f'    Saved: {onnx_path}')

    print('[2/3] Validating ONNX graph...')
    model_onnx = onnx.load(onnx_path)
    onnx.checker.check_model(model_onnx)
    print('    ONNX graph OK')


def build_trt_engine(spatial, onnx_path, engine_path, log_path):
    print(f'[3/3] Building TensorRT engine (this takes 10-40 min on Jetson Orin)...')
    print(f'    Log: {log_path}')

    # TRT 8.x limits tensor volume to 2^31-1 elements (int32 indexing).
    # At full spatial resolution the concat tensor is [B, 96, D, D, D].
    # max_batch = floor((2^31-1) / (96 * spatial^3))
    import math
    max_batch = math.floor((2**31 - 1) / (96 * spatial**3))
    max_batch = max(1, max_batch)
    print(f'    Max batch (TRT int32 limit): {max_batch}  (gradient mode batch=9 will be split)')

    shape_min = f'1x1x{spatial}x{spatial}x{spatial}'
    shape_opt = f'{max_batch}x1x{spatial}x{spatial}x{spatial}'

    cmd = [
        TRTEXEC,
        f'--onnx={onnx_path}',
        f'--saveEngine={engine_path}',
        '--fp16',
        f'--minShapes=input:{shape_min}',
        f'--optShapes=input:{shape_opt}',
        f'--maxShapes=input:{shape_opt}',
        '--memPoolSize=workspace:4096',
        '--noTF32',
        '--minTiming=1',   # 1 timing run per kernel (vs default 4) — much faster build
        '--avgTiming=1',   # 1 average timing (vs default 8)
    ]

    print('    Command:', ' '.join(cmd))

    with open(log_path, 'w') as log_f:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        log_f.write(result.stdout)
        # Also stream to terminal
        print(result.stdout[-3000:] if len(result.stdout) > 3000 else result.stdout)

    if result.returncode != 0:
        print(f'ERROR: trtexec failed (exit {result.returncode}). See {log_path}')
        sys.exit(1)

    print(f'    Engine saved: {engine_path}')
    size_mb = os.path.getsize(engine_path) / 1e6
    print(f'    Engine size: {size_mb:.1f} MB')


def main():
    args = parse_args()

    print('=== UNet3D TensorRT Export ===')
    print(f'Config  : {args.config}')
    print(f'ONNX    : {args.onnx}')
    print(f'Engine  : {args.engine}')
    print(f'Spatial : {args.spatial}³')
    print()

    model, _ = load_model_fp32(args.config)
    export_onnx(model, args.spatial, args.onnx)

    if args.no_trtexec:
        print('Skipping TensorRT build (--no-trtexec).')
        return

    if not os.path.isfile(TRTEXEC):
        print(f'ERROR: trtexec not found at {TRTEXEC}')
        sys.exit(1)

    build_trt_engine(args.spatial, args.onnx, args.engine, DEFAULT_LOG)
    print()
    print('Done. Add this line to config/test_reach.yaml to activate:')
    print(f'  trt_engine_path: "{args.engine}"')


if __name__ == '__main__':
    main()
