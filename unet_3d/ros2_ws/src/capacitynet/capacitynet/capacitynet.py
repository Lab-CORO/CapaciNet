#!/usr/bin/env python3

import os
from datetime import datetime
import rclpy
from rclpy.node import Node
import numpy as np
import torch
import time
import h5py
from curobo_msgs.msg import SparseVoxelGrid, ReachabilityMetrics
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_srvs.srv import Trigger

from .reachability_engine import ReachabilityEngine


class ReachabilityNode(Node):
    def __init__(self):
        super().__init__('reachability_node')

        self.declare_parameter('model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml')
        self.declare_parameter('save_map_path', '~/reachability_map.h5')
        self.declare_parameter('continuous_save', False)
        self.declare_parameter('save_replay_path', '')
        self.declare_parameter('log_timing', False)
        # N'affecte QUE le fallback PyTorch (pas de trt_engine_path valide dans le
        # YAML) : un engine TensorRT a sa précision figée au build (fp16 via
        # export_to_trt.py, int8 via calibrate_int8.py) et ignore ce paramètre.
        self.declare_parameter('fp16', False)

        config_path = self.get_parameter('model_config_path').value
        fp16 = bool(self.get_parameter('fp16').value)

        replay_path = self.get_parameter('save_replay_path').value
        if not replay_path:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            replay_path = os.path.expanduser(f'~/ros2_ws/src/capacitynet/results/reach_{ts}.h5')
        else:
            replay_path = os.path.expanduser(replay_path)
        os.makedirs(os.path.dirname(replay_path), exist_ok=True)
        self._replay_path = replay_path
        self._save_counter = 0
        self.get_logger().info(f'Replay save path: {self._replay_path}')

        self.engine = ReachabilityEngine(config_path, fp16=fp16)
        self.get_logger().info(f'ReachabilityEngine loaded (fp16={fp16})')

        self.metrics_pub = self.create_publisher(ReachabilityMetrics, '/reachability_node/metrics', 10)
        self._metrics_frame_counter = 0

        self.declare_parameter('point_cloud_frame_id', 'dsr01/world')
        self._pcd_frame_id = self.get_parameter('point_cloud_frame_id').value

        # Petit nuage (sous-échantillonné + seuil) -> RELIABLE, depth 1. Un publisher
        # reliable sert AUSSI un subscriber best-effort -> compatible avec tout
        # réglage QoS de RViz.
        cloud_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.rm_cloud_pub = self.create_publisher(PointCloud2, 'reachability_map', cloud_qos)

        # Réduction de taille pour un transport fluide à ~5 Hz :
        #   - voxel_stride : 1 voxel sur s par axe (s=2 -> /8 du nombre de points).
        #   - reachability_threshold : on ne garde que les voxels > seuil.
        # 128^3 dense = 32 Mo (indélivrable) ; stride 2 + seuil -> ~1-3 Mo.
        self.declare_parameter('voxel_stride', 2)
        self._stride = max(1, int(self.get_parameter('voxel_stride').value))
        self.declare_parameter('reachability_threshold', 0.001)
        self._reach_thr = float(self.get_parameter('reachability_threshold').value)

        # Figés au 1er message : indices linéaires sous-échantillonnés dans la
        # prédiction aplatie, coords correspondantes, et message PointCloud2 réutilisé.
        self._ds_lin = None          # (M,) int : indices dans pred.reshape(-1)
        self._ds_xyz = None          # (M,3) float32 : centres de voxels (fixe)
        self._grid_n = None          # N = nx*ny*nz attendu (garde-fou shape)
        self._pcd_msg = None         # PointCloud2 réutilisé
        self._pcd_dtype = np.dtype(
            [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', 'u1')])
        self._pub_count = 0

        self.save_map_srv = self.create_service(Trigger, '/reachability_node/save_map', self.handle_save_map)

        self._last_prediction_np = None
        self._last_voxel_grid_np = None
        self._last_vg_info = None
        self._pred_host = None       # buffer hôte épinglé réutilisé (shape fixe)

        # Mono-thread, publication inline dans le callback de prédiction : le nuage
        # est petit (~1-3 Mo) -> construction+publish ~quelques ms, négligeable
        # devant la prédiction (~140 ms). Pas de timer/executor multi-thread (source
        # de courses natives avec TRT/CUDA et le writer cyclone).
        self.create_subscription(
            SparseVoxelGrid,
            '/curobo_trajectory_planner/voxel_grid_sparse',
            self.handle_sparse_voxel,
            10,
        )
        self.get_logger().info('Subscribed to /curobo_trajectory_planner/voxel_grid_sparse')

    def handle_sparse_voxel(self, msg: SparseVoxelGrid):
        t_start = time.time()
        sx, sy, sz = msg.size_x, msg.size_y, msg.size_z

        voxel_map = self.engine.preprocess(msg.occupied_indices, sx, sy, sz)
        t_after_preprocess = time.time()

        prediction = self.engine.predict(voxel_map)
        t_after_predict = time.time()

        vg_info = {
            'origin': msg.origin,
            'resolution': float(msg.resolution),
            'size_x': sx, 'size_y': sy, 'size_z': sz,
        }

        # Shape constante -> buffer hôte épinglé réutilisé : pas de ré-alloc CPU
        # par frame, transfert D2H plus rapide, et copy_() caste en float32.
        if self._pred_host is None or self._pred_host.shape != prediction.shape:
            self._pred_host = torch.empty(
                prediction.shape, dtype=torch.float32, pin_memory=prediction.is_cuda)
            self._last_prediction_np = self._pred_host.numpy()
        self._pred_host.copy_(prediction)   # D2H + cast dans le buffer réutilisé

        self._last_voxel_grid_np = voxel_map.squeeze().float().cpu().numpy()
        self._last_vg_info = vg_info

        # Publication inline (nuage petit -> coût négligeable).
        # self._publish_reachability_map()

        if self.get_parameter('continuous_save').value:
            try:
                self._append_to_hdf5(self._replay_path, self._save_counter)
                self._save_counter += 1
            except Exception as e:
                self.get_logger().error(f'Continuous save failed: {e}')

        t_end = time.time()

        m = ReachabilityMetrics()
        m.header.stamp = self.get_clock().now().to_msg()
        m.frame_id = self._metrics_frame_counter
        m.t_scatter_ms = (t_after_preprocess - t_start) * 1000.0
        m.t_resize_ms = 0.0
        m.t_gpu_sync_ms = 0.0
        m.t_prediction_ms = (t_after_predict - t_after_preprocess) * 1000.0
        m.t_cpu_transfer_ms = 0.0
        m.t_total_ms = (t_end - t_start) * 1000.0
        m.fps = 1000.0 / m.t_total_ms if m.t_total_ms > 0 else 0.0
        self.metrics_pub.publish(m)
        self._metrics_frame_counter += 1

        if self.get_parameter('log_timing').value:
            self.get_logger().info(
                f"preprocess={m.t_scatter_ms:.1f}ms predict={m.t_prediction_ms:.1f}ms "
                f"total={m.t_total_ms:.1f}ms fps={m.fps:.1f}"
            )

    def _build_pcd_template(self, info, shape):
        """Pré-calcule (une fois) les indices/coords sous-échantillonnés + le message."""
        nx, ny, nz = shape
        res = float(info['resolution'])
        ox, oy, oz = float(info['origin'].x), float(info['origin'].y), float(info['origin'].z)
        s = self._stride

        ii = np.arange(0, nx, s)
        jj = np.arange(0, ny, s)
        kk = np.arange(0, nz, s)
        I, J, K = np.meshgrid(ii, jj, kk, indexing='ij')   # chacun (mx,my,mz)

        # Indices linéaires (ordre C) dans pred.reshape(-1).
        self._ds_lin = (I * ny * nz + J * nz + K).ravel().astype(np.int64)
        # Centres de voxels correspondants (fixes).
        Xc = ox + (I + 0.5) * res
        Yc = oy + (J + 0.5) * res
        Zc = oz + (K + 0.5) * res
        self._ds_xyz = np.stack(
            [Xc.ravel(), Yc.ravel(), Zc.ravel()], axis=1).astype('<f4')
        self._grid_n = nx * ny * nz

        # Message réutilisable. Intensité uint8 -> point_step = 13.
        msg = PointCloud2()
        msg.header.frame_id = self._pcd_frame_id
        msg.height = 1
        msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.UINT8,   count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 13
        msg.is_dense = True
        self._pcd_msg = msg

        self.get_logger().info(
            f'PointCloud2 template figé: grille {nx}x{ny}x{nz}, stride={s} -> '
            f'{self._ds_lin.size} voxels max, seuil={self._reach_thr}, '
            f'intensité uint8, frame={self._pcd_frame_id}')

    def _publish_reachability_map(self):
        """Publie la RM (sous-échantillonnée + seuil) en PointCloud2, intensité uint8."""
        pred = self._last_prediction_np
        if pred is None:
            return
        if self._ds_xyz is None:
            self._build_pcd_template(self._last_vg_info, pred.shape)
        if pred.size != self._grid_n:
            self.get_logger().warn(
                'Taille de grille différente du gabarit figé; nuage non publié.',
                throttle_duration_sec=5.0)
            return

        vals = pred.reshape(-1)[self._ds_lin]   # reachability des voxels échantillonnés
        mask = vals > self._reach_thr
        n = int(mask.sum())

        sel = self._ds_xyz[mask]
        out = np.empty(n, dtype=self._pcd_dtype)
        out['x'] = sel[:, 0]
        out['y'] = sel[:, 1]
        out['z'] = sel[:, 2]
        out['intensity'] = np.clip(vals[mask] * 255.0, 0, 255).astype(np.uint8)

        msg = self._pcd_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width = n
        msg.row_step = msg.point_step * n
        msg.data = out.tobytes()   # bytes -> chemin rapide rclpy (jamais un ndarray)
        self.rm_cloud_pub.publish(msg)

        self._pub_count += 1
        self.get_logger().info(
            f'RM publiée #{self._pub_count}: {n} pts ({n * 13 / 1e6:.2f} Mo)',
            throttle_duration_sec=2.0)

    def _save_to_hdf5(self, path: str) -> None:
        pred = self._last_prediction_np.astype(np.float64)
        vg = self._last_voxel_grid_np.astype(np.float64)
        info = self._last_vg_info
        res = float(info['resolution'])
        ox = float(info['origin'].x)
        oy = float(info['origin'].y)
        oz = float(info['origin'].z)
        nx, ny, nz = pred.shape

        path = os.path.expanduser(path)
        with h5py.File(path, 'w') as f:
            grp = f.create_group('/group/0')
            for name, data in [('reachability_map', pred), ('voxel_grid', vg)]:
                ds = grp.create_dataset(name, data=data)
                ds.attrs['voxel_size'] = res
                ds.attrs['origine_x'] = ox
                ds.attrs['origine_y'] = oy
                ds.attrs['origine_z'] = oz
                ds.attrs['voxel_grid_size_x'] = float(nx)
                ds.attrs['voxel_grid_size_y'] = float(ny)
                ds.attrs['voxel_grid_size_z'] = float(nz)

    def _append_to_hdf5(self, path: str, index: int) -> None:
        pred = self._last_prediction_np.astype(np.float64)
        vg = self._last_voxel_grid_np.astype(np.float64)
        info = self._last_vg_info
        res = float(info['resolution'])
        ox = float(info['origin'].x)
        oy = float(info['origin'].y)
        oz = float(info['origin'].z)
        nx, ny, nz = pred.shape

        with h5py.File(path, 'a') as f:
            grp = f.require_group(f'/group/{index}')
            for name, data in [('reachability_map', pred), ('voxel_grid', vg)]:
                if name in grp:
                    del grp[name]
                ds = grp.create_dataset(name, data=data)
                ds.attrs['voxel_size'] = res
                ds.attrs['origine_x'] = ox
                ds.attrs['origine_y'] = oy
                ds.attrs['origine_z'] = oz
                ds.attrs['voxel_grid_size_x'] = float(nx)
                ds.attrs['voxel_grid_size_y'] = float(ny)
                ds.attrs['voxel_grid_size_z'] = float(nz)

    def handle_save_map(self, request, response):
        if self._last_prediction_np is None:
            response.success = False
            response.message = 'No inference result yet.'
            return response
        path = self.get_parameter('save_map_path').value
        try:
            self._save_to_hdf5(path)
            self.get_logger().info(f'Reachability map saved to {path}')
            response.success = True
            response.message = f'Saved to {path}'
        except Exception as e:
            self.get_logger().error(f'Failed to save HDF5: {e}')
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
