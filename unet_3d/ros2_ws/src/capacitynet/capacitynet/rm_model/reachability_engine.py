#!/usr/bin/env python3

import os
import torch
import yaml
from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils


class ReachabilityEngine:
    """
    Pure inference class for reachability prediction using UNet3D (+ optional TRT).

    No ROS dependencies — usable by any node or script.
    """

    def __init__(self, config_path: str, fp16: bool = False):
        config = yaml.safe_load(open(config_path, 'r'))
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # Encodage de l'entrée, doit matcher le transformer `raw` du yaml ayant
        # servi à entraîner le checkpoint chargé :
        #   True  -> min-max vers [-1, 1]  (transform `Normalize`, ex. RM_model_cbr)
        #   False -> occupation brute [0, 1] (transform `Identity`)
        # ⚠️ Feeder [0,1] à un modèle entraîné en Normalize (ou l'inverse) dégrade
        # fortement la prédiction (corr ~0.73 au lieu de ~0.999).
        self.normalize_input = bool(config.get('normalize_input', True))

        model = get_model(config['model'])
        utils.load_checkpoint(config['model_path'], model)
        model.to(self.device)
        model.eval()

        # N'affecte QUE le fallback PyTorch : si un trt_engine_path valide est
        # fourni plus bas, la précision de l'engine TRT (fp16/int8) prime et ce
        # flag est ignoré pour l'inférence (self._infer préfère self.trt_model).
        self.use_fp16 = bool(fp16 and self.device == 'cuda')
        if self.use_fp16:
            model = model.half()
        if self.device == 'cuda':
            # Force cuDNN to select fastest algorithm; avoids fallback on Jetson/Tegra.
            torch.backends.cudnn.benchmark = True

        self.model = model

        self.trt_model = None
        trt_engine_path = config.get('trt_engine_path', None)
        if trt_engine_path and os.path.isfile(trt_engine_path):
            try:
                from .trt_model import TRTModel
                self.trt_model = TRTModel(trt_engine_path)
                self.trt_model.warmup(spatial=128)
                torch.cuda.synchronize()
            except Exception:
                self.trt_model = None

    def preprocess(self, occupied_indices, size_x: int, size_y: int, size_z: int) -> torch.Tensor:
        """Convert sparse voxel indices to a dense 5D tensor ready for inference.

        Args:
            occupied_indices: linear indices of occupied voxels (list or 1D tensor)
            size_x, size_y, size_z: voxel grid dimensions

        Returns:
            Tensor of shape (1, 1, size_x, size_y, size_z) on device, FP16 or FP32.
        """
        occ = torch.tensor(occupied_indices, dtype=torch.long, device=self.device)
        raw = torch.zeros(size_x * size_y * size_z, dtype=torch.int32, device=self.device)
        if occ.numel() > 0:
            raw[occ] = 1
        raw = raw.reshape(size_x, size_y, size_z)
        # Occupation dans la convention d'entraînement : obstacle=1, free=0.
        occupancy = raw.float()
        if self.normalize_input:
            # obstacle->+1, free->-1  (= Normalize(raw) à l'entraînement, ex. cbr).
            x = self._normalize(occupancy)
        else:
            # occupation brute dans [0, 1]  (checkpoints entraînés en Identity).
            x = occupancy
        dtype = torch.float16 if self.use_fp16 else torch.float32
        return x.to(dtype).unsqueeze(0).unsqueeze(0)

    @staticmethod
    def _normalize(tensor: torch.Tensor) -> torch.Tensor:
        """Normalize to [-1, 1]: (x - min) / (max - min + eps) scaled to [-1, 1]."""
        mn = tensor.min()
        mx = tensor.max()
        norm01 = (tensor - mn) / (mx - mn + 1e-10)
        return torch.clamp(2.0 * norm01 - 1.0, -1.0, 1.0)

    def _infer(self, x: torch.Tensor) -> torch.Tensor:
        if self.trt_model is not None:
            return self.trt_model.infer(x)
        return self.model(x)

    def predict(self, voxel_map_5d: torch.Tensor) -> torch.Tensor:
        """Run inference on a single voxel map.

        Args:
            voxel_map_5d: Tensor of shape (1, 1, sx, sy, sz)

        Returns:
            Tensor of shape (sx, sy, sz)
        """
        with torch.no_grad():
            result = self._infer(voxel_map_5d)
            if self.device == 'cuda':
                torch.cuda.synchronize()
        return result.squeeze()

    def predict_batch(self, voxel_maps: torch.Tensor) -> torch.Tensor:
        """Run inference on a batch of voxel maps.

        Args:
            voxel_maps: Tensor of shape (N, 1, sx, sy, sz)

        Returns:
            Tensor of shape (N, sx, sy, sz)
        """
        with torch.no_grad():
            result = self._infer(voxel_maps)
            if self.device == 'cuda':
                torch.cuda.synchronize()
        return result.squeeze(1)
