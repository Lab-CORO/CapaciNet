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
from geometry_msgs.msg import Point
from curobo_msgs.srv import GetVoxelGrid
from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils




class ReachabilityNode(Node):
    def __init__(self):
        super().__init__('reachability_node')

        # Service client
        self.voxel_client = self.create_client(GetVoxelGrid, '/curobo_gen_traj/get_voxel_grid')
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Voxel service not available, waiting...')

        self.req = GetVoxelGrid.Request()
        self.latest_voxel = None
        self.latest_prediction = None

        # Publisher: PointCloud2
        self.pc_pub = self.create_publisher(PointCloud2, '/reachability_map_pc', 10)

        # Load UNet3D model
        config_path = "/workspace/capacitynet/config/test_reach.yaml"
        config = yaml.safe_load(open(config_path, 'r'))
        self.model = get_model(config['model'])
        utils.load_checkpoint(config['model_path'], self.model)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.model.eval()

        # Timers
        self.create_timer(0.5, self.call_voxel_service)
        self.create_timer(1.0, self.publish_prediction_pc)

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

        # Convert to PyTorch tensor: [B, C, D, H, W]
        voxel_map_ts = torch.from_numpy(voxel_map.astype(np.float32)).unsqueeze(0).unsqueeze(0)
        voxel_map_ts = voxel_map_ts.to(self.device, non_blocking=True)

        # UNet prediction
        with torch.no_grad():
            prediction = self.model(voxel_map_ts)


        # Store latest voxel & prediction
        self.latest_voxel = vg
        self.latest_prediction = prediction.squeeze().cpu().numpy()  # remove batch & channel
        
    def publish_prediction_pc(self):
        """Publish UNet3D prediction as PointCloud2."""
        if self.latest_prediction is None or self.latest_voxel is None:
            return

        vg = self.latest_voxel
        pred = self.latest_prediction
        points = []

        # Iterate only over occupied voxels
        dx, dy, dz = pred.shape
        for x in range(dx):
            for y in range(dy):
                for z in range(dz):
                    if pred[x, y, z] > 0.5:  # threshold
                        px = vg.origin.x + x * vg.resolutions.x
                        py = vg.origin.y + y * vg.resolutions.y
                        pz = vg.origin.z + z * vg.resolutions.z
                        points.append([px, py, pz])

        if not points:
            return  # nothing to publish

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_0"

        pc2_msg = pc2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(pc2_msg)
        self.get_logger().info(f"Published point cloud with {len(points)} points")


def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
