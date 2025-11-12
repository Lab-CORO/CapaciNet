#!/usr/bin/env python3



# # To run this script run this command first:
# # export PYTHONPATH=/opt/conda/lib/python3.10/site-packages:$PYTHONPATH
# # Verify with this: echo $PYTHONPATH

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import yaml
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, Point32
from curobo_msgs.srv import GetVoxelGrid
from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils
import threading
from queue import Queue
import open3d as o3d
from reachability_map_visualizer.msg import WorkSpace, WsSphere
import h5py
import os
from datetime import datetime




class ReachabilityNode(Node):
    def __init__(self):
        super().__init__('reachability_node')

        # Service client
        self.voxel_client = self.create_client(GetVoxelGrid, '/curobo_gen_traj/get_voxel_grid')
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Voxel service not available, waiting...')

        self.req = GetVoxelGrid.Request()

        # Publisher: WorkSpace message
        self.ws_pub = self.create_publisher(WorkSpace, '/reachability_map', 10)

        # Load UNet3D model
        config_path = "/workspace/capacitynet/config/test_reach.yaml"
        config = yaml.safe_load(open(config_path, 'r'))
        self.model = get_model(config['model'])
        utils.load_checkpoint(config['model_path'], self.model)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.model.eval()

        # CUDA stream for async GPU->CPU transfer
        if self.device == 'cuda':
            self.cuda_stream = torch.cuda.Stream()
        else:
            self.cuda_stream = None

        # Queue for async CPU processing (voxel->pointcloud conversion)
        self.conversion_queue = Queue(maxsize=2)  # double buffering

        # Start background thread for point cloud conversion
        self.conversion_thread = threading.Thread(target=self._conversion_worker, daemon=True)
        self.conversion_thread.start()

        # For publishing
        self.latest_workspace_msg = None
        self.lock = threading.Lock()

        # HDF5 saving
        self.hdf5_save_counter = 0
        self.hdf5_filepath = None
        self.hdf5_lock = threading.Lock()

        # Timers - 10Hz for prediction pipeline
        self.create_timer(0.1, self.call_voxel_service)  # 10Hz
        self.create_timer(0.1, self.publish_prediction_pc)  # 10Hz

    def call_voxel_service(self):
        """Call the voxel grid service and store latest voxel grid."""
        future = self.voxel_client.call_async(self.req)
        future.add_done_callback(self.handle_voxel_response)

    def handle_voxel_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        vg = response.voxel_grid
        # Convert to NumPy and invert occupancy
        voxel_map = np.frombuffer(bytes(vg.data), dtype=np.uint32)
        voxel_map = 1 - voxel_map.reshape(vg.size_x, vg.size_y, vg.size_z)

        # Check and adjust resolution if needed
        current_resolution = float(vg.resolutions.x)  # Assuming uniform resolution
        target_resolution = 0.02  # Target resolution expected by the model

        if abs(current_resolution - target_resolution) > 1e-6:  # If resolution is different
            scale_factor = current_resolution / target_resolution
            # self.get_logger().info(f"Adjusting resolution from {current_resolution}m to {target_resolution}m (scale factor: {scale_factor:.3f})")

            # Convert to tensor for interpolation
            voxel_tensor = torch.from_numpy(voxel_map.astype(np.float32)).unsqueeze(0).unsqueeze(0)

            # Calculate new dimensions
            new_size = (
                int(vg.size_x * scale_factor),
                int(vg.size_y * scale_factor),
                int(vg.size_z * scale_factor)
            )

            # Resize using trilinear interpolation
            voxel_map_ts = torch.nn.functional.interpolate(
                voxel_tensor,
                size=new_size,
                mode='nearest',  # Use nearest neighbor for binary voxel data
                align_corners=None
            )

            # self.get_logger().info(f"Resized voxel grid from ({vg.size_x}, {vg.size_y}, {vg.size_z}) to {new_size}")

            # Create voxel grid info with updated dimensions and resolution
            vg_info = {
                'origin': vg.origin,
                'resolution': target_resolution,
                'size_x': new_size[0],
                'size_y': new_size[1],
                'size_z': new_size[2],
            }
        else:
            # No resizing needed, convert to tensor directly
            voxel_map_ts = torch.from_numpy(voxel_map.astype(np.float32)).unsqueeze(0).unsqueeze(0)

            # Create voxel grid info with original dimensions
            vg_info = {
                'origin': vg.origin,
                'resolution': current_resolution,
                'size_x': vg.size_x,
                'size_y': vg.size_y,
                'size_z': vg.size_z,
            }

        # Move to device
        voxel_map_ts = voxel_map_ts.to(self.device, non_blocking=True)

        # UNet prediction
        with torch.no_grad():
            prediction = self.model(voxel_map_ts)
            prediction = prediction.squeeze()  # remove batch & channel, keep on GPU

        # Push to conversion queue (non-blocking)
        try:
            self.conversion_queue.put_nowait((prediction, voxel_map_ts, vg_info))
        except:
            pass  # Queue full, skip this frame to maintain real-time
        
    def _conversion_worker(self):
        """Background thread: converts GPU prediction -> CPU WorkSpace message."""
        while True:
            try:
                # Blocking get from queue
                prediction_gpu, voxel_map_ts_gpu, vg_info = self.conversion_queue.get()

                # ASYNC GPU->CPU transfer using pinned memory
                if self.cuda_stream is not None:
                    with torch.cuda.stream(self.cuda_stream):
                        prediction_cpu = prediction_gpu.cpu()
                        voxel_map_ts_cpu = voxel_map_ts_gpu.cpu()
                    torch.cuda.current_stream().wait_stream(self.cuda_stream)
                else:
                    prediction_cpu = prediction_gpu.cpu()
                    voxel_map_ts_cpu = voxel_map_ts_gpu.cpu()

                pred_np = prediction_cpu.numpy()
                voxel_grid_np = 1- voxel_map_ts_cpu.squeeze().numpy()

                # Fast vectorized conversion: voxel grid -> WorkSpace message
                ws_msg = self._voxel_to_hdf5(pred_np, voxel_grid_np, vg_info, threshold=0.0)
                # ws_msg = self._voxel_to_workspace_vectorized(pred_np, vg_info, threshold=0.1)

                # if ws_msg is None:
                #     continue

                # # Thread-safe update
                # with self.lock:
                #     self.latest_workspace_msg = ws_msg

            except Exception as e:
                self.get_logger().error(f"Conversion worker error: {e}")

    def _voxel_to_workspace_vectorized(self, pred, vg_info, threshold=0.2):
        """Fast vectorized voxel->WorkSpace message conversion using numpy.

        Args:
            pred: numpy array with prediction values
            vg_info: dict with voxel grid metadata (origin, resolution, size_x/y/z)
            threshold: threshold for filtering voxels
        """
        # Find all occupied voxel indices
        occupied = np.argwhere(pred > threshold)

        if len(occupied) == 0:
            return None

        # Create WorkSpace message
        ws_msg = WorkSpace()

        # Fill header
        ws_msg.header.stamp = self.get_clock().now().to_msg()
        ws_msg.header.frame_id = "base_link"

        # Fill voxel grid info from vg_info dict
        ws_msg.resolution = float(vg_info['resolution'])
        ws_msg.size_x = int(vg_info['size_x'])
        ws_msg.size_y = int(vg_info['size_y'])
        ws_msg.size_z = int(vg_info['size_z'])

        # Set origin from vg_info
        ws_msg.origine.x = float(vg_info['origin'].x)
        ws_msg.origine.y = float(vg_info['origin'].y)
        ws_msg.origine.z = float(vg_info['origin'].z)

        # Vectorized coordinate transformation and create WsSphere messages
        resolution = vg_info['resolution']
        origin = vg_info['origin']

        for idx in occupied:
            wss = WsSphere()
            wss.header.stamp = ws_msg.header.stamp
            wss.header.frame_id = ws_msg.header.frame_id

            # Calculate world coordinates using correct resolution
            wss.point.x = float(origin.x + idx[0] * resolution)
            wss.point.y = float(origin.y + idx[1] * resolution)
            wss.point.z = float(origin.z + idx[2] * resolution)

            # Set reachability index (normalized prediction value)
            wss.ri = float(pred[idx[0], idx[1], idx[2]])*100
            # self.get_logger().info(f"{wss.ri}")

            ws_msg.ws_spheres.append(wss)

        return ws_msg


    def _voxel_to_hdf5(self, pred, voxel_grid, vg_info, threshold=0.0, output_dir="/workspace/capacitynet/reachability_maps"):
        """Save the prediction to a single HDF5 file with incremental group IDs.

        Args:
            pred: numpy array with prediction values (after interpolation)
            voxel_grid: numpy array with voxel grid data (after interpolation)
            vg_info: dict with voxel grid metadata (origin, resolution, size_x/y/z)
            threshold: threshold for filtering (not used currently)
            output_dir: directory where to save the HDF5 file

        Returns:
            tuple: (filepath, group_id) or (None, None) on failure
        """
        with self.hdf5_lock:
            try:
                # Create output directory if it doesn't exist
                os.makedirs(output_dir, exist_ok=True)

                # Create the HDF5 file on first call
                if self.hdf5_filepath is None:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    filename = f"reachability_{timestamp}.h5"
                    self.hdf5_filepath = os.path.join(output_dir, filename)
                    self.get_logger().info(f"Created HDF5 file: {self.hdf5_filepath}")

                # Open file in append mode
                with h5py.File(self.hdf5_filepath, 'a') as h5file:
                    # Create group structure with incremental ID
                    if 'group' not in h5file:
                        group = h5file.create_group('group')
                    else:
                        group = h5file['group']

                    # Create subgroup with current counter
                    subgroup_name = str(self.hdf5_save_counter)
                    subgroup = group.create_group(subgroup_name)

                    # Generate timestamp for this save
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

                    # Save reachability_map dataset with its attributes
                    reach_ds = subgroup.create_dataset('reachability_map', data=pred, compression='gzip')
                    reach_ds.attrs['origine_x'] = float(vg_info['origin'].x)
                    reach_ds.attrs['origine_y'] = float(vg_info['origin'].y)
                    reach_ds.attrs['origine_z'] = float(vg_info['origin'].z)
                    reach_ds.attrs['voxel_grid_size_x'] = int(vg_info['size_x'])
                    reach_ds.attrs['voxel_grid_size_y'] = int(vg_info['size_y'])
                    reach_ds.attrs['voxel_grid_size_z'] = int(vg_info['size_z'])
                    reach_ds.attrs['resolution'] = float(vg_info['resolution'])
                    reach_ds.attrs['timestamp'] = timestamp
                    reach_ds.attrs['frame_id'] = 'base_link'

                    # Save voxel_grid dataset with its attributes
                    voxel_ds = subgroup.create_dataset('voxel_grid', data=voxel_grid, compression='gzip')
                    voxel_ds.attrs['origine_x'] = float(vg_info['origin'].x)
                    voxel_ds.attrs['origine_y'] = float(vg_info['origin'].y)
                    voxel_ds.attrs['origine_z'] = float(vg_info['origin'].z)
                    voxel_ds.attrs['voxel_grid_size_x'] = int(vg_info['size_x'])
                    voxel_ds.attrs['voxel_grid_size_y'] = int(vg_info['size_y'])
                    voxel_ds.attrs['voxel_grid_size_z'] = int(vg_info['size_z'])
                    voxel_ds.attrs['resolution'] = float(vg_info['resolution'])
                    voxel_ds.attrs['timestamp'] = timestamp
                    voxel_ds.attrs['frame_id'] = 'base_link'

                    # Save group-level attribute
                    subgroup.attrs['group_id'] = self.hdf5_save_counter

                current_id = self.hdf5_save_counter
                self.hdf5_save_counter += 1

                return self.hdf5_filepath, current_id

            except Exception as e:
                self.get_logger().error(f"Failed to save HDF5 file: {e}")
                return None, None


    def publish_prediction_pc(self):
        """Publish latest WorkSpace message from conversion thread."""
        with self.lock:
            if self.latest_workspace_msg is not None:
                self.ws_pub.publish(self.latest_workspace_msg)
                # self.get_logger().info(f"Published WorkSpace message")  # Reduce logging for 10Hz


def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
