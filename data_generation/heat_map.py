#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import h5py
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import std_msgs.msg

import matplotlib.pyplot as plt

# Path to the HDF5 file
H5_FILE_PATH = "/home/ros2_ws/install/data_generation/share/data_generation/data/test_0_5_vae.h5"

class VoxelMapPublisher(Node):
    def __init__(self):
        super().__init__('voxel_map_publisher')

        # Create a publisher for the voxel map using CUBE_LIST
        self.publisher = self.create_publisher(Marker, 'voxel_map', 10)

        # Timer to publish data
        self.timer = self.create_timer(1.0, self.publish_voxel_map)

        # Load voxel data from the HDF5 file
        self.voxel_map = self.load_voxel_map()
        self.voxel_data = []
        id = 0

    def load_voxel_map(self):
        """Loads voxel data from the HDF5 file and accumulates voxel intensity."""
        voxel_dict = {}

        with h5py.File(H5_FILE_PATH, 'r') as h5_file:
            groups = h5_file['group']

            for group_name in groups:
                self.voxel_data = np.array(groups[group_name]['voxel_map']['data'])

                for voxel in self.voxel_data:
                    key = tuple(voxel[:3])  # (x, y, z) coordinates as key
                    
                    voxel_dict[key] = voxel_dict.get(key, 0) + (1-voxel[3])  # Increase intensity
                
        return voxel_dict

    def publish_voxel_map(self):
        """Creates and publishes a single CUBE_LIST marker for RViz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "voxel_map"
        marker.id = 0
        marker.type = Marker.CUBE_LIST  # Using CUBE_LIST for efficiency
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.pose.orientation.w = 1.0  # Identity quaternion
        intensities = []
        for (x, y, z), intensity in self.voxel_map.items():
            intensities.append(intensity)
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            marker.points.append(point)

            # Color based on intensity (normalized)
            color_intensity = min(intensity / 955, 1.0)  # Normalize (cap at 1.0)
            marker.color.r = (0.0, 1.0)[intensity == 1.0]
            marker.color.g = 0.0
            marker.color.b = 0.0 #- color_intensity
            marker.color.a = 0.5  # Fully opaque

        self.publisher.publish(marker)
        self.get_logger().info(f"Published {len(marker.points)} voxels.")
        # print(intensities)
        # input("Press Enter to continue...")
        # with matplotlib plot a graph with the number of occupy voxel (color intensity) and free voxel
        # plt.hist(intensities, bins=100, density=False, alpha=0.6, color='b')
        # plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = VoxelMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
