#!/usr/bin/env python3

import os
from datetime import datetime
import rclpy
from rclpy.node import Node
import numpy as np
import time
import h5py
from curobo_msgs.msg import SparseVoxelGrid, ReachabilityMetrics
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

        config_path = self.get_parameter('model_config_path').value

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

        self.engine = ReachabilityEngine(config_path)
        self.get_logger().info('ReachabilityEngine loaded')

        self.metrics_pub = self.create_publisher(ReachabilityMetrics, '/reachability_node/metrics', 10)
        self._metrics_frame_counter = 0

        self.save_map_srv = self.create_service(Trigger, '/reachability_node/save_map', self.handle_save_map)

        self._last_prediction_np = None
        self._last_voxel_grid_np = None
        self._last_vg_info = None

        self.create_subscription(
            SparseVoxelGrid,
            '/unified_planner/voxel_grid_sparse',
            self.handle_sparse_voxel,
            10,
        )
        self.get_logger().info('Subscribed to /unified_planner/voxel_grid_sparse')

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

        self._last_prediction_np = prediction.float().cpu().numpy()
        self._last_voxel_grid_np = voxel_map.squeeze().float().cpu().numpy()
        self._last_vg_info = vg_info

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
