#!/usr/bin/env python3



# # To run this script run this command first:
# # export PYTHONPATH=/opt/conda/lib/python3.10/site-packages:$PYTHONPATH
# # Verify with this: echo $PYTHONPATH

import os
import rclpy
from rclpy.node import Node
import numpy as np
import torch
import yaml
import time
from curobo_msgs.srv import GetVoxelGrid
from pytorch3dunet.unet3d.model import get_model
from pytorch3dunet.unet3d import utils
from reachability_map_visualizer.msg import WorkSpace
from geometry_msgs.msg import Twist, PointStamped

from .obstacle_transformer import ObstacleMapTransformer
from .gradient_controller import GradientBasedController




class ReachabilityNode(Node):
    def __init__(self):
        super().__init__('reachability_node')

        # Declare gradient control parameters
        self.declare_parameter('enable_gradient_control', True)
        self.declare_parameter('grid_spacing', 0.10)
        self.declare_parameter('control_gain', 1.0)
        self.declare_parameter('max_linear_velocity', 0.10)
        self.declare_parameter('workspace_radius', 0.30)

        # Declare static obstacles parameters
        self.declare_parameter('use_static_obstacles', False)
        self.declare_parameter('static_obstacles_yaml', '/workspace/capacitynet/config/floor_world.yml')

        # Declare workspace center topic
        self.declare_parameter('workspace_center_topic', '/workspace_center')

        # Declare debug parameters
        self.declare_parameter('log_control_timing', True)
        self.declare_parameter('log_quality_scores', False)

        # Service client  /unified_planner/
        self.voxel_client = self.create_client(GetVoxelGrid, '/unified_planner/get_voxel_grid')
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Voxel service not available, waiting...')

        self.req = GetVoxelGrid.Request()

        # Publisher: WorkSpace message
        self.ws_pub = self.create_publisher(WorkSpace, '/reachability_map', 10)

        # Load UNet3D model
        config_path = "/home/ros2_ws/src/capacitynet/config/test_reach.yaml"
        config = yaml.safe_load(open(config_path, 'r'))
        self.model = get_model(config['model'])
        utils.load_checkpoint(config['model_path'], self.model)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.model.eval()

        # Use FP16 for faster inference and lower VRAM usage
        if self.device == 'cuda':
            self.model = self.model.half()
            self.use_fp16 = True
            self.get_logger().info("Using FP16 (half precision) for faster inference")
            # Force cuDNN to benchmark and select the fastest supported algorithm.
            # Avoids CUDNN_STATUS_NOT_SUPPORTED fallback on Jetson/embedded GPUs.
            torch.backends.cudnn.benchmark = True
            # torch.compile requires Triton which is unavailable on Jetson (ARM/Tegra).
        else:
            self.use_fp16 = False

        # TensorRT engine — optional, graceful fallback to PyTorch if absent or broken.
        self.trt_model = None
        trt_engine_path = config.get('trt_engine_path', None)
        if trt_engine_path and os.path.isfile(trt_engine_path):
            try:
                from .trt_model import TRTModel
                self.trt_model = TRTModel(trt_engine_path)
                self.trt_model.warmup(spatial=152)
                torch.cuda.synchronize()
                self.get_logger().info(f"TRT engine loaded and warmed up: {trt_engine_path}")
            except Exception as e:
                self.get_logger().warn(f"TRT engine load failed ({e}), falling back to PyTorch")
                self.trt_model = None
        else:
            self.get_logger().info("No TRT engine configured, using PyTorch inference")

        # Get gradient control parameters
        enable_control = self.get_parameter('enable_gradient_control').value
        grid_spacing = self.get_parameter('grid_spacing').value
        gain = self.get_parameter('control_gain').value
        max_vel = self.get_parameter('max_linear_velocity').value
        workspace_radius = self.get_parameter('workspace_radius').value

        # Initialize obstacle transformer
        use_static_obs = self.get_parameter('use_static_obstacles').value
        static_yaml = self.get_parameter('static_obstacles_yaml').value if use_static_obs else None

        self.obstacle_transformer = ObstacleMapTransformer(
            resolution=0.02,  # Will be updated dynamically
            device=self.device,
            static_obstacles_yaml=static_yaml
        )
        self.get_logger().info("ObstacleMapTransformer initialized")

        # Initialize gradient controller
        self.gradient_controller = GradientBasedController(
            workspace_radius=workspace_radius,
            grid_spacing=grid_spacing,
            gain=gain,
            max_linear_vel=max_vel,
            device=self.device
        )
        self.get_logger().info("GradientBasedController initialized")

        # Initialize workspace center (will be updated by subscriber)
        self.workspace_center = None

        # Subscribe to workspace center topic
        workspace_topic = self.get_parameter('workspace_center_topic').value
        self.workspace_center_sub = self.create_subscription(
            PointStamped,
            workspace_topic,
            self.on_workspace_center_received,
            10
        )
        self.get_logger().info(f"Subscribed to workspace center: {workspace_topic}")

        # Publisher for velocity commands
        if enable_control:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.get_logger().info("Gradient control enabled, publishing to /cmd_vel")
        else:
            self.cmd_vel_pub = None
            self.get_logger().info("Gradient control disabled")

        # Flags for logging
        self.log_control_timing = self.get_parameter('log_control_timing').value
        self.log_quality_scores = self.get_parameter('log_quality_scores').value

        # Timer - 0.1Hz for prediction and publishing pipeline
        self.create_timer(10.0, self.call_voxel_service)  # 10Hz

    def _run_inference(self, x: torch.Tensor) -> torch.Tensor:
        if self.trt_model is not None:
            return self.trt_model.infer(x)
        return self.model(x)

    def call_voxel_service(self):
        """Call the voxel grid service and store latest voxel grid."""
        future = self.voxel_client.call_async(self.req)
        future.add_done_callback(self.handle_voxel_response)

    def handle_voxel_response(self, future):
        t_start = time.time()

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        t_after_response = time.time()

        vg = response.voxel_grid
        # frombuffer on a bytes object always yields a read-only array, and CUDA
        # doesn't support uint32 arithmetic — reinterpret as int32 (same bit pattern
        # for values 0/1) and copy once to get a writable buffer for torch.from_numpy.
        voxel_map = np.frombuffer(bytes(vg.data), dtype=np.int32).reshape(
            vg.size_x, vg.size_y, vg.size_z
        ).copy()

        t_after_conversion = time.time()

        current_resolution = float(vg.resolutions.x)
        target_resolution = 0.02  # Target resolution expected by the model

        # Transfer to GPU as int32 with a single copy, then do inversion/cast there.
        # On Jetson (unified memory) this still copies due to CUDA cache attributes,
        # but we avoid multiple CPU-side intermediate allocations.
        raw_gpu = torch.from_numpy(voxel_map).to(self.device, non_blocking=True)

        if abs(current_resolution - target_resolution) > 1e-6:
            scale_factor = current_resolution / target_resolution
            new_size = (
                int(vg.size_x * scale_factor),
                int(vg.size_y * scale_factor),
                int(vg.size_z * scale_factor),
            )
            # Invert occupancy, cast to FP16/FP32, resize — all on GPU
            dtype = torch.float16 if self.use_fp16 else torch.float32
            voxel_map_ts = torch.nn.functional.interpolate(
                (1 - raw_gpu).to(dtype).unsqueeze(0).unsqueeze(0),
                size=new_size,
                mode='nearest',
            )
            vg_info = {
                'origin': vg.origin,
                'resolution': target_resolution,
                'size_x': new_size[0],
                'size_y': new_size[1],
                'size_z': new_size[2],
            }
        else:
            dtype = torch.float16 if self.use_fp16 else torch.float32
            voxel_map_ts = (1 - raw_gpu).to(dtype).unsqueeze(0).unsqueeze(0)
            vg_info = {
                'origin': vg.origin,
                'resolution': current_resolution,
                'size_x': vg.size_x,
                'size_y': vg.size_y,
                'size_z': vg.size_z,
            }

        t_after_resize = time.time()

        if self.device == 'cuda':
            torch.cuda.synchronize()

        t_after_gpu_transfer = time.time()

        # Update transformer resolution if changed
        self.obstacle_transformer.update_resolution(vg_info['resolution'])

        # Check if gradient control is enabled
        enable_control = self.get_parameter('enable_gradient_control').value

        if enable_control:
            # Check if workspace center has been received
            if self.workspace_center is None:
                self.get_logger().warn("Workspace center not yet received, skipping gradient control")
                enable_control = False

        if enable_control:
            # Generate 9 transformed voxel maps for gradient computation
            transformed_voxels = self.obstacle_transformer.generate_grid_transforms(
                voxel_map_ts,
                grid_spacing=self.gradient_controller.delta,
                origin=vg_info['origin']
            )  # Shape: (9, 1, size_x, size_y, size_z)

            if self.device == 'cuda':
                torch.cuda.synchronize()
            t_after_transforms = time.time()

            # UNet prediction on all 9 voxel maps (batch)
            with torch.no_grad():
                predictions = self._run_inference(transformed_voxels)  # (9, 1, size_x, size_y, size_z)
                if self.device == 'cuda':
                    torch.cuda.synchronize()  # Ensure GPU is done before capturing timer
                t_after_prediction = time.time()
                predictions = predictions.squeeze(1)  # (9, size_x, size_y, size_z)

            t_after_squeeze = time.time()

            # Compute gradient and control command
            twist_msg, debug_info = self.gradient_controller.compute_control(
                predictions,
                vg_info
            )
            t_after_control = time.time()

            # Publish velocity command
            if self.cmd_vel_pub is not None:
                self.cmd_vel_pub.publish(twist_msg)
            t_after_cmd_publish = time.time()

            # Use center RM (index 4) for workspace visualization
            prediction_center = predictions[4]  # Current position RM

            # Log control info
            if self.log_quality_scores:
                scores = debug_info['scores']
                self.get_logger().info(
                    f"Scores: [{scores[0]:.3f} {scores[1]:.3f} {scores[2]:.3f}] "
                    f"[{scores[3]:.3f} {scores[4]:.3f} {scores[5]:.3f}] "
                    f"[{scores[6]:.3f} {scores[7]:.3f} {scores[8]:.3f}]"
                )

            grad_x, grad_y = debug_info['gradient']
            vx, vy = debug_info['velocity']
            self.get_logger().info(
                f"∇Q=({grad_x:+.4f},{grad_y:+.4f}) | "
                f"v=({vx:+.4f},{vy:+.4f}) m/s"
            )

        else:
            # Single RM prediction (original behavior)
            with torch.no_grad():
                prediction = self._run_inference(voxel_map_ts)  # (1, 1, size_x, size_y, size_z)
                if self.device == 'cuda':
                    torch.cuda.synchronize()  # Ensure GPU is done before capturing timer
                t_after_prediction = time.time()
                prediction = prediction.squeeze()  # (size_x, size_y, size_z)

            prediction_center = prediction
            t_after_transforms = t_after_gpu_transfer  # No transforms
            t_after_squeeze = t_after_prediction
            t_after_control = t_after_prediction
            t_after_cmd_publish = t_after_prediction

        # Convert center RM to CPU for publishing
        if self.use_fp16:
            prediction_cpu = prediction_center.float().cpu().numpy()
        else:
            prediction_cpu = prediction_center.cpu().numpy()

        t_after_cpu_transfer = time.time()

        # Create and publish WorkSpace message
        # ws_msg = self._create_workspace_message(prediction_cpu, vg_info)
        t_after_msg_creation = time.time()

        # self.ws_pub.publish(ws_msg)
        t_after_publish = time.time()

        # Release local references — PyTorch's own memory pool handles reclamation.
        # Never call empty_cache() in a real-time loop: it destroys the pool and
        # forces re-allocation on the next call (~130ms penalty per iteration).
        if enable_control:
            del predictions
            del transformed_voxels
        else:
            del prediction
        del voxel_map_ts
        del raw_gpu

        t_end = time.time()

        # Enhanced timing log
        if self.log_control_timing or enable_control:
            timing_msg = (
                f"Timing [ms]: "
                f"Response: {(t_after_response - t_start)*1000:.1f} | "
                f"Conversion: {(t_after_conversion - t_after_response)*1000:.1f} | "
                f"Resize: {(t_after_resize - t_after_conversion)*1000:.1f} | "
                f"GPU Xfer: {(t_after_gpu_transfer - t_after_resize)*1000:.1f} | "
            )

            if enable_control:
                timing_msg += (
                    f"Transforms: {(t_after_transforms - t_after_gpu_transfer)*1000:.1f} | "
                    f"Prediction(9x): {(t_after_prediction - t_after_transforms)*1000:.1f} | "
                    f"Control: {(t_after_control - t_after_squeeze)*1000:.1f} | "
                    f"Cmd Pub: {(t_after_cmd_publish - t_after_control)*1000:.1f} | "
                )
            else:
                timing_msg += f"Prediction: {(t_after_prediction - t_after_gpu_transfer)*1000:.1f} | "

            timing_msg += (
                f"CPU Xfer: {(t_after_cpu_transfer - t_after_cmd_publish)*1000:.1f} | "
                f"Msg: {(t_after_msg_creation - t_after_cpu_transfer)*1000:.1f} | "
                f"Publish: {(t_after_publish - t_after_msg_creation)*1000:.1f} | "
                f"Cleanup: {(t_end - t_after_publish)*1000:.1f} | "
                f"TOTAL: {(t_end - t_start)*1000:.1f}"
            )

            self.get_logger().info(timing_msg)
        
    def _create_workspace_message(self, pred_np, vg_info):
        """Create WorkSpace message using optimized flat array format.

        Args:
            pred_np: numpy array with prediction values (shape: size_x, size_y, size_z)
            vg_info: dict with voxel grid metadata (origin, resolution, size_x/y/z)

        Returns:
            WorkSpace message with ri_values as flat array
        """
        ws_msg = WorkSpace()

        # Fill header
        ws_msg.header.stamp = self.get_clock().now().to_msg()
        ws_msg.header.frame_id = "base_0"

        # Fill voxel grid info
        ws_msg.resolution = float(vg_info['resolution'])
        ws_msg.size_x = int(vg_info['size_x'])
        ws_msg.size_y = int(vg_info['size_y'])
        ws_msg.size_z = int(vg_info['size_z'])

        # Set origin
        ws_msg.origine.x = float(vg_info['origin'].x)
        ws_msg.origine.y = float(vg_info['origin'].y)
        ws_msg.origine.z = float(vg_info['origin'].z)

        # Flatten prediction array and convert to list
        # Indexed as: ri_values[x * size_y * size_z + y * size_z + z]
        ws_msg.ri = (pred_np * 100.0).flatten().astype(np.float32).tolist()

        return ws_msg

    def on_workspace_center_received(self, msg: PointStamped):
        """
        Callback for workspace center topic (ArTag position).

        Args:
            msg: PointStamped message with x, y, z position
        """
        workspace_center = (
            msg.point.x,
            msg.point.y,
            msg.point.z
        )
        self.workspace_center = workspace_center
        self.gradient_controller.update_workspace_center(workspace_center)

        # Log only on first reception
        if not hasattr(self, '_last_logged_center'):
            self.get_logger().info(
                f"Workspace center received: ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})"
            )
            self._last_logged_center = workspace_center


def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
