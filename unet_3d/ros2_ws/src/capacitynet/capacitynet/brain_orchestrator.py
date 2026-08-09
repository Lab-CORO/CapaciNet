#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum

from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from ros2_markertracker_interfaces.msg import FiducialMarkerArray

from control_msgs.action import GripperCommand
from curobo_msgs.srv import TrajectoryGeneration
from curobo_msgs.msg import SparseVoxelGrid

from .base_commander import gradient_to_velocity


class State(Enum):
    IDLE = 0
    WAITING_FOR_TAG = 1
    MOVING_TO_PICK = 2
    GRASPING = 3
    MOVING_TO_PLACE = 4
    RELEASING = 5
    RETURNING_HOME = 6
    ERROR = 7


class BrainOrchestrator(Node):
    """
    Nœud orchestrateur pour séquence pick-and-place + contrôle gradient base mobile.

    Le gradient control (ReachabilityEngine + ObstacleMapTransformer + GradientBasedController)
    tourne dans le même process via import direct — pas de latence inter-process.
    """

    def __init__(self):
        super().__init__('brain_orchestrator')

        self._load_parameters()

        self.current_state = State.IDLE
        self.previous_state = None
        self.ar_tag_pose = None
        self.ar_tag_last_seen = None
        self.workspace_center = None

        self._setup_interfaces()

        if self.enable_gradient_control:
            self._setup_gradient_control()

        self.timer = self.create_timer(
            1.0 / self.state_check_frequency,
            self.state_machine_update
        )

        self.get_logger().info("Brain Orchestrator initialized")
        self.get_logger().info(f"  - AR tag topic: {self.ar_tag_topic}")
        self.get_logger().info(f"  - Trigger topic: {self.trigger_topic}")
        self.get_logger().info(f"  - Trajectory service: {self.trajectory_service}")
        self.get_logger().info(f"  - Gripper action: {self.gripper_action}")
        self.get_logger().info(f"  - Gradient control: {'enabled' if self.enable_gradient_control else 'disabled'}")

    def _load_parameters(self):
        # AR Tag detection
        self.declare_parameter('ar_tag_topic', '/fiducial_markers')
        self.declare_parameter('ar_tag_timeout', 10.0)
        self.declare_parameter('ar_tag_id', -1)

        # Trigger
        self.declare_parameter('trigger_topic', '/brain/trigger')

        # Trajectory generation service
        self.declare_parameter('trajectory_service', '/curobo_trajectory_planner/generate_trajectory')

        # Gripper control
        self.declare_parameter('gripper_action', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 0.08)
        self.declare_parameter('gripper_max_effort', 50.0)

        # Pick position offsets
        self.declare_parameter('pick_approach_offset_z', 0.15)
        self.declare_parameter('pick_grasp_offset_z', 0.05)

        # Place position
        self.declare_parameter('place_position_x', 0.5)
        self.declare_parameter('place_position_y', 0.3)
        self.declare_parameter('place_position_z', 0.2)
        self.declare_parameter('place_offset_z', 0.10)

        # Home position
        self.declare_parameter('home_position_x', 0.0)
        self.declare_parameter('home_position_y', 0.0)
        self.declare_parameter('home_position_z', 0.5)

        # Timing
        self.declare_parameter('state_check_frequency', 10.0)
        self.declare_parameter('trajectory_timeout', 30.0)
        self.declare_parameter('gripper_timeout', 5.0)

        # Gradient control.
        # Superseded by the standalone `gradient_base_controller` node, which owns
        # /cmd_vel. Leaving this on as well makes two nodes fight over the base.
        # Set True only to compare against this legacy in-process path.
        self.declare_parameter('enable_gradient_control', False)
        self.declare_parameter('model_config_path', '/home/ros2_ws/src/capacitynet/config/test_reach.yaml')
        self.declare_parameter('grid_spacing', 0.10)
        self.declare_parameter('control_gain', 1.0)
        self.declare_parameter('max_linear_velocity', 0.10)
        self.declare_parameter('workspace_radius', 0.30)
        self.declare_parameter('workspace_frame', 'dsr01/base_link')
        self.declare_parameter('use_static_obstacles', False)
        self.declare_parameter('static_obstacles_yaml', '/home/ros2_ws/src/capacitynet/config/floor_world.yml')
        self.declare_parameter('log_control_timing', True)
        self.declare_parameter('log_quality_scores', False)

        self.ar_tag_topic = self.get_parameter('ar_tag_topic').value
        self.ar_tag_timeout = self.get_parameter('ar_tag_timeout').value
        self.ar_tag_id = self.get_parameter('ar_tag_id').value
        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.trajectory_service = self.get_parameter('trajectory_service').value
        self.gripper_action = self.get_parameter('gripper_action').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_closed_position = self.get_parameter('gripper_closed_position').value
        self.gripper_max_effort = self.get_parameter('gripper_max_effort').value
        self.pick_approach_offset_z = self.get_parameter('pick_approach_offset_z').value
        self.pick_grasp_offset_z = self.get_parameter('pick_grasp_offset_z').value
        self.place_position_x = self.get_parameter('place_position_x').value
        self.place_position_y = self.get_parameter('place_position_y').value
        self.place_position_z = self.get_parameter('place_position_z').value
        self.place_offset_z = self.get_parameter('place_offset_z').value
        self.home_position_x = self.get_parameter('home_position_x').value
        self.home_position_y = self.get_parameter('home_position_y').value
        self.home_position_z = self.get_parameter('home_position_z').value
        self.state_check_frequency = self.get_parameter('state_check_frequency').value
        self.trajectory_timeout = self.get_parameter('trajectory_timeout').value
        self.gripper_timeout = self.get_parameter('gripper_timeout').value
        self.enable_gradient_control = self.get_parameter('enable_gradient_control').value
        self.workspace_radius = self.get_parameter('workspace_radius').value
        self.workspace_frame = self.get_parameter('workspace_frame').value
        self.log_control_timing = self.get_parameter('log_control_timing').value
        self.log_quality_scores = self.get_parameter('log_quality_scores').value

    def _setup_interfaces(self):
        self.trigger_sub = self.create_subscription(Bool, self.trigger_topic, self.on_trigger_received, 10)
        self.ar_tag_sub = self.create_subscription(FiducialMarkerArray, self.ar_tag_topic, self.on_ar_tag_received, 10)

        self.traj_gen_client = self.create_client(TrajectoryGeneration, self.trajectory_service)
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action)

        # Workspace visualization: sphere marker for RViz (refreshed by state machine).
        self.workspace_marker_pub = self.create_publisher(Marker, '/workspace_marker', 10)

        self.get_logger().info("Waiting for trajectory service...")
        if self.traj_gen_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Trajectory service available")
        else:
            self.get_logger().warn("Trajectory service not available yet")

        self.get_logger().info("Waiting for gripper action server...")
        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Gripper action server available")
        else:
            self.get_logger().warn("Gripper action server not available yet")

    def _setup_gradient_control(self):
        from .reachability_engine import ReachabilityEngine
        from .obstacle_transformer import ObstacleMapTransformer
        from .gradient_controller import GradientBasedController
        import torch

        config_path = self.get_parameter('model_config_path').value
        grid_spacing = self.get_parameter('grid_spacing').value
        gain = self.get_parameter('control_gain').value
        max_vel = self.get_parameter('max_linear_velocity').value
        workspace_radius = self.workspace_radius
        use_static_obs = self.get_parameter('use_static_obstacles').value
        static_yaml = self.get_parameter('static_obstacles_yaml').value if use_static_obs else None
        device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.engine = ReachabilityEngine(config_path)
        self.obstacle_transformer = ObstacleMapTransformer(
            resolution=0.02,
            device=device,
            static_obstacles_yaml=static_yaml
        )
        self.gradient_ctrl = GradientBasedController(
            workspace_radius=workspace_radius,
            grid_spacing=grid_spacing,
            device=device
        )
        # Gain and saturation are motion decisions, applied via gradient_to_velocity.
        self.control_gain = gain
        self.max_linear_vel = max_vel

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voxel_grid_sub = self.create_subscription(
            SparseVoxelGrid,
            '/curobo_trajectory_planner/voxel_grid_sparse',
            self._on_voxel_grid,
            10
        )
        self.get_logger().info("Gradient control ready, publishing to /cmd_vel")

    def _on_voxel_grid(self, msg: SparseVoxelGrid):
        if self.workspace_center is None:
            return

        t_start = time.time()
        sx, sy, sz = msg.size_x, msg.size_y, msg.size_z
        vg_info = {
            'origin': msg.origin,
            'resolution': float(msg.resolution),
            'size_x': sx, 'size_y': sy, 'size_z': sz,
        }

        voxel_map = self.engine.preprocess(msg.occupied_indices, sx, sy, sz)

        self.obstacle_transformer.update_resolution(vg_info['resolution'])
        transformed = self.obstacle_transformer.generate_grid_transforms(
            voxel_map, grid_spacing=self.gradient_ctrl.delta, origin=vg_info['origin']
        )

        rm_9 = self.engine.predict_batch(transformed)

        self.gradient_ctrl.update_workspace_center(self.workspace_center)
        debug_info = self.gradient_ctrl.evaluate(rm_9, vg_info)
        grad_x, grad_y = debug_info['gradient']
        vx, vy = gradient_to_velocity(
            grad_x, grad_y, self.control_gain, self.max_linear_vel)

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        self.cmd_vel_pub.publish(twist)

        del rm_9, transformed, voxel_map

        if self.log_control_timing:
            t_total = (time.time() - t_start) * 1000.0
            self.get_logger().info(
                f"∇Q=({grad_x:+.4f},{grad_y:+.4f}) v=({vx:+.4f},{vy:+.4f}) m/s total={t_total:.1f}ms"
            )

        if self.log_quality_scores:
            scores = debug_info['scores']
            self.get_logger().info(
                f"Scores: [{scores[0]:.3f} {scores[1]:.3f} {scores[2]:.3f}] "
                f"[{scores[3]:.3f} {scores[4]:.3f} {scores[5]:.3f}] "
                f"[{scores[6]:.3f} {scores[7]:.3f} {scores[8]:.3f}]"
            )

    # ── State machine ──────────────────────────────────────────────────────────

    def state_machine_update(self):
        # Continuously refresh the workspace sphere so RViz tracks its current center.
        self.publish_workspace_marker()

        if self.current_state != self.previous_state:
            self.get_logger().info(
                f"State: {self.previous_state.name if self.previous_state else 'None'} → {self.current_state.name}"
            )
            self.previous_state = self.current_state

        if self.current_state == State.IDLE:
            self._state_idle()
        elif self.current_state == State.WAITING_FOR_TAG:
            self._state_waiting_for_tag()
        elif self.current_state == State.MOVING_TO_PICK:
            self._state_moving_to_pick()
        elif self.current_state == State.GRASPING:
            self._state_grasping()
        elif self.current_state == State.MOVING_TO_PLACE:
            self._state_moving_to_place()
        elif self.current_state == State.RELEASING:
            self._state_releasing()
        elif self.current_state == State.RETURNING_HOME:
            self._state_returning_home()
        elif self.current_state == State.ERROR:
            self._state_error()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def on_trigger_received(self, msg: Bool):
        if msg.data and self.current_state == State.IDLE:
            self.get_logger().info("Trigger received, starting pick-and-place sequence")
            self.transition_to(State.WAITING_FOR_TAG)

    def on_ar_tag_received(self, msg: FiducialMarkerArray):
        self.ar_tag_pose = msg.marker[0]
        self.ar_tag_last_seen = self.get_clock().now()
        # if self.current_state in [State.WAITING_FOR_TAG, State.MOVING_TO_PICK]:
        self.set_workspace_center(
            self.ar_tag_pose.pose_cov_stamped.pose.pose.position.x,
            self.ar_tag_pose.pose_cov_stamped.pose.pose.position.y,
            self.ar_tag_pose.pose_cov_stamped.pose.pose.position.z
        )

    # ── States ────────────────────────────────────────────────────────────────

    def _state_idle(self):
        self.set_workspace_center(self.home_position_x, self.home_position_y, self.home_position_z)

    def _state_waiting_for_tag(self):
        if self.ar_tag_pose is not None:
            self.get_logger().info("AR tag detected, moving to pick")
            self.transition_to(State.MOVING_TO_PICK)
        else:
            if not hasattr(self, 'waiting_start_time'):
                self.waiting_start_time = self.get_clock().now()
                self.get_logger().info(f"Waiting for AR tag (timeout: {self.ar_tag_timeout}s)...")
            elapsed = (self.get_clock().now() - self.waiting_start_time).nanoseconds / 1e9
            if elapsed > self.ar_tag_timeout:
                self.get_logger().error("AR tag detection timeout")
                self.transition_to(State.ERROR)

    def _state_moving_to_pick(self):
        if not hasattr(self, 'pick_trajectory_sent'):
            target_pose = Pose()
            target_pose.position.x = self.ar_tag_pose.pose.position.x
            target_pose.position.y = self.ar_tag_pose.pose.position.y
            target_pose.position.z = self.ar_tag_pose.pose.position.z + self.pick_approach_offset_z
            target_pose.orientation = self.ar_tag_pose.pose.orientation
            success = self.call_trajectory_service(target_pose)
            if success:
                self.pick_trajectory_sent = True
                self.get_logger().info("Trajectory to pick executed, moving to GRASPING")
                self.transition_to(State.GRASPING)
            else:
                self.get_logger().error("Failed to generate trajectory to pick")
                self.transition_to(State.ERROR)

    def _state_grasping(self):
        if not hasattr(self, 'grasp_executed'):
            if not hasattr(self, 'descent_executed'):
                target_pose = Pose()
                target_pose.position.x = self.ar_tag_pose.pose.position.x
                target_pose.position.y = self.ar_tag_pose.pose.position.y
                target_pose.position.z = self.ar_tag_pose.pose.position.z + self.pick_grasp_offset_z
                target_pose.orientation = self.ar_tag_pose.pose.orientation
                self.get_logger().info("Descending to grasp position...")
                success = self.call_trajectory_service(target_pose)
                if success:
                    self.descent_executed = True
                else:
                    self.transition_to(State.ERROR)
                    return
            self.get_logger().info("Closing gripper...")
            success = self.call_gripper_action(self.gripper_closed_position, self.gripper_max_effort)
            if success:
                self.grasp_executed = True
                self.get_logger().info("Object grasped, moving to MOVING_TO_PLACE")
                self.transition_to(State.MOVING_TO_PLACE)
            else:
                self.get_logger().error("Failed to grasp object")
                self.transition_to(State.ERROR)

    def _state_moving_to_place(self):
        if not hasattr(self, 'place_trajectory_sent'):
            self.set_workspace_center(self.place_position_x, self.place_position_y, self.place_position_z)
            target_pose = Pose()
            target_pose.position.x = self.place_position_x
            target_pose.position.y = self.place_position_y
            target_pose.position.z = self.place_position_z + self.place_offset_z
            target_pose.orientation.w = 1.0
            success = self.call_trajectory_service(target_pose)
            if success:
                self.place_trajectory_sent = True
                self.get_logger().info("Trajectory to place executed, moving to RELEASING")
                self.transition_to(State.RELEASING)
            else:
                self.get_logger().error("Failed to generate trajectory to place")
                self.transition_to(State.ERROR)

    def _state_releasing(self):
        if not hasattr(self, 'release_executed'):
            self.get_logger().info("Opening gripper...")
            success = self.call_gripper_action(self.gripper_open_position, self.gripper_max_effort)
            if success:
                self.release_executed = True
                self.get_logger().info("Object released, moving to RETURNING_HOME")
                self.transition_to(State.RETURNING_HOME)
            else:
                self.get_logger().error("Failed to release object")
                self.transition_to(State.ERROR)

    def _state_returning_home(self):
        if not hasattr(self, 'home_trajectory_sent'):
            self.set_workspace_center(self.home_position_x, self.home_position_y, self.home_position_z)
            target_pose = Pose()
            target_pose.position.x = self.home_position_x
            target_pose.position.y = self.home_position_y
            target_pose.position.z = self.home_position_z
            target_pose.orientation.w = 1.0
            success = self.call_trajectory_service(target_pose)
            if success:
                self.home_trajectory_sent = True
                self.get_logger().info("Returned to home, sequence complete!")
                self.transition_to(State.IDLE)
            else:
                self.get_logger().error("Failed to return home")
                self.transition_to(State.ERROR)

    def _state_error(self):
        self.get_logger().error("Sequence failed, returning to IDLE")
        self.transition_to(State.IDLE)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def transition_to(self, new_state: State):
        self.current_state = new_state
        if new_state == State.WAITING_FOR_TAG:
            self.ar_tag_pose = None
            if hasattr(self, 'waiting_start_time'):
                delattr(self, 'waiting_start_time')
        elif new_state == State.IDLE:
            self.ar_tag_pose = None
            for attr in ['waiting_start_time', 'pick_trajectory_sent', 'descent_executed',
                         'grasp_executed', 'place_trajectory_sent', 'release_executed',
                         'home_trajectory_sent']:
                if hasattr(self, attr):
                    delattr(self, attr)

    def set_workspace_center(self, x: float, y: float, z: float):
        self.workspace_center = (x, y, z)

    def publish_workspace_marker(self):
        """Publish the spherical workspace as an RViz SPHERE marker.

        The sphere is centered on the current workspace_center and its diameter
        equals 2 * workspace_radius, matching the spherical region actually
        scored by WorkspaceEvaluation. No-op until a workspace center is set.
        """
        if self.workspace_center is None:
            return

        x, y, z = self.workspace_center
        diameter = 2.0 * self.workspace_radius

        marker = Marker()
        marker.header.frame_id = "rgb_camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3
        self.workspace_marker_pub.publish(marker)

    def call_trajectory_service(self, target_pose: Pose) -> bool:
        request = TrajectoryGeneration.Request()
        request.target_pose = target_pose
        self.get_logger().info(
            f"Calling trajectory service: target=({target_pose.position.x:.2f}, "
            f"{target_pose.position.y:.2f}, {target_pose.position.z:.2f})"
        )
        future = self.traj_gen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.trajectory_timeout)
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Trajectory succeeded: {response.message}")
                return True
            else:
                self.get_logger().error(f"Trajectory failed: {response.message}")
                return False
        else:
            self.get_logger().error("Trajectory service timed out")
            return False

    def call_gripper_action(self, position: float, max_effort: float) -> bool:
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        self.get_logger().info(f"Gripper command: position={position:.3f}, effort={max_effort:.1f}")
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=self.gripper_timeout)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.gripper_timeout)
        result = result_future.result().result
        self.get_logger().info(
            f"Gripper done: position={result.position:.3f}, reached_goal={result.reached_goal}"
        )
        return result.reached_goal


def main(args=None):
    rclpy.init(args=args)
    node = BrainOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
