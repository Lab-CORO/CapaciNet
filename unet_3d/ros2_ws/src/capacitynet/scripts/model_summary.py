#!/usr/bin/env python3
"""Display UNet3D architecture summary using torchinfo (or fallback to print).

Usage:
    python3 scripts/model_summary.py
    python3 scripts/model_summary.py --spatial 64
    python3 scripts/model_summary.py --config /path/to/config.yaml

Prerequisites (optional, for detailed per-layer table):
    pip install torchinfo
"""

import argparse
import sys
import os

import torch
import yaml

sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')
from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils

CONFIG_DIR = '/home/ros2_ws/src/capacitynet/config'
DEFAULT_CONFIG = os.path.join(CONFIG_DIR, 'test_reach.yaml')


def parse_args():
    p = argparse.ArgumentParser(description='Display UNet3D model summary')
    p.add_argument('--config', default=DEFAULT_CONFIG)
    p.add_argument('--spatial', type=int, default=128,
                   help='Spatial dimension (cubic) for the input size.')
    return p.parse_args()


def load_model_fp32(config_path):
    config = yaml.safe_load(open(config_path, 'r'))
    model = get_model(config['model'])
    utils.load_checkpoint(config['model_path'], model)
    model.eval()
    return model, config


def main():
    args = parse_args()

    print('=== UNet3D Model Summary ===')
    print(f'Config  : {args.config}')
    print(f'Spatial : {args.spatial}³')
    print()

    model, config = load_model_fp32(args.config)
    model_cfg = config['model']
    print(f"Architecture : {model_cfg.get('name')}")
    print(f"layer_order  : {model_cfg.get('layer_order')}")
    print(f"f_maps       : {model_cfg.get('f_maps')}")
    print(f"in/out ch    : {model_cfg.get('in_channels')} → {model_cfg.get('out_channels')}")
    print(f"checkpoint   : {config['model_path']}")
    print()

    total_params = sum(p.numel() for p in model.parameters())
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)

    try:
        from torchinfo import summary
        input_size = (1, 1, args.spatial, args.spatial, args.spatial)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        summary(model.to(device), input_size=input_size,
                col_names=('input_size', 'output_size', 'num_params', 'mult_adds'),
                depth=4, device=device)
    except ImportError:
        print('torchinfo not installed — showing model structure only.')
        print('Install with:  pip install torchinfo\n')
        print(model)
        print()
        print(f'Total params    : {total_params:,}')
        print(f'Trainable params: {trainable_params:,}')
        print(f'Non-trainable   : {total_params - trainable_params:,}')


if __name__ == '__main__':
    main()
