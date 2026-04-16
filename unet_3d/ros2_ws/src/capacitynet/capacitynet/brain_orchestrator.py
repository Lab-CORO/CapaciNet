#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum

from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from std_msgs.msg import Bool
from control_msgs.action import GripperCommand
from curobo_msgs.srv import TrajectoryGeneration


class State(Enum):
    """États de la machine à états."""
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
    Nœud orchestrateur pour séquence pick-and-place.

    Coordonne:
    - Détection AR tag
    - Génération de trajectoire (curobo)
    - Contrôle gripper
    - Publication workspace center
    """

    def __init__(self):
        super().__init__('brain_orchestrator')

        # Load parameters
        self._load_parameters()

        # State machine
        self.current_state = State.IDLE
        self.previous_state = None

        # Data storage
        self.ar_tag_pose = None
        self.ar_tag_last_seen = None

        # Create clients and subscribers
        self._setup_interfaces()

        # Timer for state machine updates
        self.timer = self.create_timer(
            1.0 / self.state_check_frequency,
            self.state_machine_update
        )

        self.get_logger().info("Brain Orchestrator initialized")
        self.get_logger().info(f"  - AR tag topic: {self.ar_tag_topic}")
        self.get_logger().info(f"  - Trigger topic: {self.trigger_topic}")
        self.get_logger().info(f"  - Workspace center topic: {self.workspace_center_topic}")
        self.get_logger().info(f"  - Trajectory service: {self.trajectory_service}")
        self.get_logger().info(f"  - Gripper action: {self.gripper_action}")

    def _load_parameters(self):
        """Load all ROS2 parameters."""
        # AR Tag detection
        self.declare_parameter('ar_tag_topic', '/ar_pose_marker')
        self.declare_parameter('ar_tag_timeout', 10.0)
        self.declare_parameter('ar_tag_id', -1)

        # Trigger
        self.declare_parameter('trigger_topic', '/brain/trigger')

        # Workspace center publishing
        self.declare_parameter('workspace_center_topic', '/workspace_center')

        # Trajectory generation service
        self.declare_parameter('trajectory_service', '/unified_planner/generate_trajectory')

        # Gripper control
        self.declare_parameter('gripper_action', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 0.08)
        self.declare_parameter('gripper_max_effort', 50.0)

        # Pick position offsets (relative to AR tag)
        self.declare_parameter('pick_approach_offset_z', 0.15)
        self.declare_parameter('pick_grasp_offset_z', 0.05)

        # Place position (absolute coordinates)
        self.declare_parameter('place_position_x', 0.5)
        self.declare_parameter('place_position_y', 0.3)
        self.declare_parameter('place_position_z', 0.2)
        self.declare_parameter('place_offset_z', 0.10)

        # Home position (absolute coordinates)
        self.declare_parameter('home_position_x', 0.0)
        self.declare_parameter('home_position_y', 0.0)
        self.declare_parameter('home_position_z', 0.5)

        # Timing
        self.declare_parameter('state_check_frequency', 10.0)
        self.declare_parameter('trajectory_timeout', 30.0)
        self.declare_parameter('gripper_timeout', 5.0)

        # Get parameter values
        self.ar_tag_topic = self.get_parameter('ar_tag_topic').value
        self.ar_tag_timeout = self.get_parameter('ar_tag_timeout').value
        self.ar_tag_id = self.get_parameter('ar_tag_id').value

        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.workspace_center_topic = self.get_parameter('workspace_center_topic').value
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

    def _setup_interfaces(self):
        """Create publishers, subscribers, service clients, action clients."""
        # Subscribers
        self.trigger_sub = self.create_subscription(
            Bool,
            self.trigger_topic,
            self.on_trigger_received,
            10
        )

        self.ar_tag_sub = self.create_subscription(
            PoseStamped,
            self.ar_tag_topic,
            self.on_ar_tag_received,
            10
        )

        # Publishers
        self.workspace_center_pub = self.create_publisher(
            PointStamped,
            self.workspace_center_topic,
            10
        )

        # Service clients
        self.traj_gen_client = self.create_client(
            TrajectoryGeneration,
            self.trajectory_service
        )

        # Action clients
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action
        )

        self.get_logger().info("Waiting for trajectory service...")
        if self.traj_gen_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Trajectory service available")
        else:
            self.get_logger().warn("Trajectory service not available yet (will retry later)")

        self.get_logger().info("Waiting for gripper action server...")
        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Gripper action server available")
        else:
            self.get_logger().warn("Gripper action server not available yet (will retry later)")

    def state_machine_update(self):
        """Main state machine loop, called at state_check_frequency."""
        # Log state transitions
        if self.current_state != self.previous_state:
            self.get_logger().info(f"State: {self.previous_state.name if self.previous_state else 'None'} → {self.current_state.name}")
            self.previous_state = self.current_state

        # Execute state logic
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

    # Callbacks
    def on_trigger_received(self, msg: Bool):
        """Callback pour trigger de démarrage."""
        if msg.data and self.current_state == State.IDLE:
            self.get_logger().info("Trigger received, starting pick-and-place sequence")
            self.transition_to(State.WAITING_FOR_TAG)

    def on_ar_tag_received(self, msg: PoseStamped):
        """Callback pour réception AR tag."""
        self.ar_tag_pose = msg
        self.ar_tag_last_seen = self.get_clock().now()

        # Publish workspace center at AR tag location
        if self.current_state in [State.WAITING_FOR_TAG, State.MOVING_TO_PICK]:
            self.publish_workspace_center(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )

    # State implementations
    def _state_idle(self):
        """IDLE: Waiting for trigger."""
        # Publish home position as workspace center
        self.publish_workspace_center(
            self.home_position_x,
            self.home_position_y,
            self.home_position_z
        )

    def _state_waiting_for_tag(self):
        """WAITING_FOR_TAG: Wait for AR tag detection."""
        if self.ar_tag_pose is not None:
            self.get_logger().info("AR tag detected, moving to pick")
            self.transition_to(State.MOVING_TO_PICK)
        else:
            # Check timeout
            if not hasattr(self, 'waiting_start_time'):
                self.waiting_start_time = self.get_clock().now()
                self.get_logger().info(f"Waiting for AR tag (timeout: {self.ar_tag_timeout}s)...")

            elapsed = (self.get_clock().now() - self.waiting_start_time).nanoseconds / 1e9
            if elapsed > self.ar_tag_timeout:
                self.get_logger().error("AR tag detection timeout")
                self.transition_to(State.ERROR)

    def _state_moving_to_pick(self):
        """MOVING_TO_PICK: Generate and execute trajectory to pick position."""
        if not hasattr(self, 'pick_trajectory_sent'):
            # Compute pre-grasp pose (above object)
            target_pose = Pose()
            target_pose.position.x = self.ar_tag_pose.pose.position.x
            target_pose.position.y = self.ar_tag_pose.pose.position.y
            target_pose.position.z = self.ar_tag_pose.pose.position.z + self.pick_approach_offset_z

            # Copy orientation from AR tag
            target_pose.orientation = self.ar_tag_pose.pose.orientation

            # Call trajectory service
            success = self.call_trajectory_service(target_pose)

            if success:
                self.pick_trajectory_sent = True
                self.get_logger().info("Trajectory to pick position executed, moving to GRASPING")
                self.transition_to(State.GRASPING)
            else:
                self.get_logger().error("Failed to generate trajectory to pick")
                self.transition_to(State.ERROR)

    def _state_grasping(self):
        """GRASPING: Close gripper to grasp object."""
        if not hasattr(self, 'grasp_executed'):
            # First, descend to grasp position
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

            # Then close gripper
            self.get_logger().info("Closing gripper to grasp object...")
            success = self.call_gripper_action(
                self.gripper_closed_position,
                self.gripper_max_effort
            )

            if success:
                self.grasp_executed = True
                self.get_logger().info("Object grasped, moving to MOVING_TO_PLACE")
                self.transition_to(State.MOVING_TO_PLACE)
            else:
                self.get_logger().error("Failed to grasp object")
                self.transition_to(State.ERROR)

    def _state_moving_to_place(self):
        """MOVING_TO_PLACE: Move to place position."""
        if not hasattr(self, 'place_trajectory_sent'):
            # Publish workspace center at place position
            self.publish_workspace_center(
                self.place_position_x,
                self.place_position_y,
                self.place_position_z
            )

            # Compute place pose (with offset above)
            target_pose = Pose()
            target_pose.position.x = self.place_position_x
            target_pose.position.y = self.place_position_y
            target_pose.position.z = self.place_position_z + self.place_offset_z

            # Use default orientation (pointing down)
            target_pose.orientation.w = 1.0
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0

            success = self.call_trajectory_service(target_pose)

            if success:
                self.place_trajectory_sent = True
                self.get_logger().info("Trajectory to place position executed, moving to RELEASING")
                self.transition_to(State.RELEASING)
            else:
                self.get_logger().error("Failed to generate trajectory to place")
                self.transition_to(State.ERROR)

    def _state_releasing(self):
        """RELEASING: Open gripper to release object."""
        if not hasattr(self, 'release_executed'):
            self.get_logger().info("Opening gripper to release object...")
            success = self.call_gripper_action(
                self.gripper_open_position,
                self.gripper_max_effort
            )

            if success:
                self.release_executed = True
                self.get_logger().info("Object released, moving to RETURNING_HOME")
                self.transition_to(State.RETURNING_HOME)
            else:
                self.get_logger().error("Failed to release object")
                self.transition_to(State.ERROR)

    def _state_returning_home(self):
        """RETURNING_HOME: Return to home position."""
        if not hasattr(self, 'home_trajectory_sent'):
            # Publish workspace center at home
            self.publish_workspace_center(
                self.home_position_x,
                self.home_position_y,
                self.home_position_z
            )

            target_pose = Pose()
            target_pose.position.x = self.home_position_x
            target_pose.position.y = self.home_position_y
            target_pose.position.z = self.home_position_z
            target_pose.orientation.w = 1.0
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0

            success = self.call_trajectory_service(target_pose)

            if success:
                self.home_trajectory_sent = True
                self.get_logger().info("Returned to home, sequence complete!")
                self.get_logger().info("="*60)
                self.transition_to(State.IDLE)
            else:
                self.get_logger().error("Failed to return home")
                self.transition_to(State.ERROR)

    def _state_error(self):
        """ERROR: Error state, log and return to IDLE."""
        self.get_logger().error("Sequence failed, returning to IDLE")
        self.get_logger().error("="*60)
        self.transition_to(State.IDLE)

    # Helper methods
    def transition_to(self, new_state: State):
        """Transition to new state."""
        self.current_state = new_state

        # Reset state-specific variables
        if new_state == State.WAITING_FOR_TAG:
            self.ar_tag_pose = None
            if hasattr(self, 'waiting_start_time'):
                delattr(self, 'waiting_start_time')
        elif new_state == State.IDLE:
            # Reset all sequence data
            self.ar_tag_pose = None
            # Clean up all state-specific attributes
            for attr in ['waiting_start_time', 'pick_trajectory_sent', 'descent_executed',
                        'grasp_executed', 'place_trajectory_sent', 'release_executed',
                        'home_trajectory_sent']:
                if hasattr(self, attr):
                    delattr(self, attr)

    def publish_workspace_center(self, x: float, y: float, z: float):
        """Publish workspace center for gradient controller."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_0'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        self.workspace_center_pub.publish(msg)

    def call_trajectory_service(self, target_pose: Pose) -> bool:
        """Call trajectory generation service."""
        request = TrajectoryGeneration.Request()
        request.target_pose = target_pose

        self.get_logger().info(
            f"Calling trajectory service: target=({target_pose.position.x:.2f}, "
            f"{target_pose.position.y:.2f}, {target_pose.position.z:.2f})"
        )

        future = self.traj_gen_client.call_async(request)

        # Wait for response (with timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.trajectory_timeout)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Trajectory generation succeeded: {response.message}")
                return True
            else:
                self.get_logger().error(f"Trajectory generation failed: {response.message}")
                return False
        else:
            self.get_logger().error("Trajectory service call timed out")
            return False

    def call_gripper_action(self, position: float, max_effort: float) -> bool:
        """Send gripper command via action."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f"Sending gripper command: position={position:.3f}, effort={max_effort:.1f}")

        send_goal_future = self.gripper_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=self.gripper_timeout)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        self.get_logger().info("Gripper goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.gripper_timeout)

        result = result_future.result().result
        self.get_logger().info(
            f"Gripper action completed: position={result.position:.3f}, "
            f"reached_goal={result.reached_goal}"
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
