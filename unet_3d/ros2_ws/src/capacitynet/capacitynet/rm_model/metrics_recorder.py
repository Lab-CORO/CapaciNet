#!/usr/bin/env python3
"""Record ReachabilityMetrics to a CSV file with GPU stats polled via nvidia-smi.

Launch alongside the reachability_node, then Ctrl-C when done.
The CSV is flushed after every row so data is preserved even on hard kill.

Usage
-----
ros2 run capacitynet metrics_recorder \
    --ros-args -p output_csv:=~/my_run.csv -p gpu_poll_rate_hz:=2.0
"""

import csv
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from curobo_msgs.msg import ReachabilityMetrics


_NVIDIA_SMI = '/usr/sbin/nvidia-smi'
_CSV_FIELDS = [
    'timestamp',
    'frame_id',
    't_scatter_ms',
    't_resize_ms',
    't_gpu_sync_ms',
    't_prediction_ms',
    't_cpu_transfer_ms',
    't_total_ms',
    'fps',
    'gpu_util_pct',
    'gpu_mem_used_mb',
    'gpu_mem_total_mb',
]


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')

        self.declare_parameter('output_csv', '~/reachability_metrics.csv')
        self.declare_parameter('gpu_poll_rate_hz', 2.0)

        output_path = os.path.expanduser(
            self.get_parameter('output_csv').value)
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

        self._csv_file = open(output_path, 'w', newline='')
        self._writer = csv.DictWriter(self._csv_file, fieldnames=_CSV_FIELDS)
        self._writer.writeheader()
        self._csv_file.flush()

        # GPU stats — written by background thread, read in ROS callback.
        self._gpu_util: float = 0.0
        self._gpu_mem_used: float = 0.0
        self._gpu_mem_total: float = 0.0
        self._gpu_lock = threading.Lock()

        poll_rate = float(self.get_parameter('gpu_poll_rate_hz').value)
        self._poll_interval = 1.0 / max(poll_rate, 0.1)
        self._stop_event = threading.Event()
        self._gpu_thread = threading.Thread(
            target=self._gpu_poll_loop, daemon=True, name='gpu_poll')
        self._gpu_thread.start()

        self.create_subscription(
            ReachabilityMetrics,
            '/reachability_node/metrics',
            self._on_metrics,
            100,
        )

        self.get_logger().info(
            f'MetricsRecorder: saving to {output_path}  '
            f'(GPU poll {poll_rate:.1f} Hz)')

    # ------------------------------------------------------------------
    # GPU polling
    # ------------------------------------------------------------------

    def _gpu_poll_loop(self):
        while not self._stop_event.is_set():
            try:
                result = subprocess.run(
                    [_NVIDIA_SMI,
                     '--query-gpu=utilization.gpu,memory.used,memory.total',
                     '--format=csv,noheader,nounits'],
                    capture_output=True, text=True, timeout=2.0,
                )
                if result.returncode == 0:
                    parts = result.stdout.strip().split(',')
                    if len(parts) == 3:
                        util = float(parts[0].strip())
                        used = float(parts[1].strip())
                        total = float(parts[2].strip())
                        with self._gpu_lock:
                            self._gpu_util = util
                            self._gpu_mem_used = used
                            self._gpu_mem_total = total
            except Exception as exc:
                self.get_logger().debug(f'nvidia-smi poll failed: {exc}')
            time.sleep(self._poll_interval)

    # ------------------------------------------------------------------
    # Metrics subscriber callback
    # ------------------------------------------------------------------

    def _on_metrics(self, msg: ReachabilityMetrics):
        with self._gpu_lock:
            gpu_util = self._gpu_util
            gpu_mem_used = self._gpu_mem_used
            gpu_mem_total = self._gpu_mem_total

        self._writer.writerow({
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.frame_id,
            't_scatter_ms': round(msg.t_scatter_ms, 3),
            't_resize_ms': round(msg.t_resize_ms, 3),
            't_gpu_sync_ms': round(msg.t_gpu_sync_ms, 3),
            't_prediction_ms': round(msg.t_prediction_ms, 3),
            't_cpu_transfer_ms': round(msg.t_cpu_transfer_ms, 3),
            't_total_ms': round(msg.t_total_ms, 3),
            'fps': round(msg.fps, 3),
            'gpu_util_pct': gpu_util,
            'gpu_mem_used_mb': gpu_mem_used,
            'gpu_mem_total_mb': gpu_mem_total,
        })
        self._csv_file.flush()

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._stop_event.set()
        self._gpu_thread.join(timeout=2.0)
        self._csv_file.close()
        self.get_logger().info('MetricsRecorder: CSV closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
