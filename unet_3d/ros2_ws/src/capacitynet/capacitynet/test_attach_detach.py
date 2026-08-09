#!/usr/bin/env python3
"""Test node for curobo's attach/detach services.

Self-contained: on a Bool trigger it runs a full cycle against the curobo
unified planner —

  add_object (a cuboid) -> attach_object -> hold -> detach_object -> remove_object

logging each service's success/message. Lets you exercise attach/detach
repeatedly without the full grasp pipeline.

Async pattern mirrors capacitynet/grasp.py: coroutine callbacks + `await
client.call_async(req)` (never spin_until_future_complete inside a callback).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, ColorRGBA
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Vector3
from curobo_msgs.srv import AddObject, AttachObject, RemoveObject
from curobo_msgs.action import SendTrajectory


class AttachDetachTester(Node):
    """Drive curobo add/attach/detach/remove on a Bool trigger."""

    def __init__(self):
        super().__init__('attach_detach_tester')

        self._load_parameters()
        self._busy = False
        self._setup_interfaces()

        self.get_logger().info("Attach/detach tester initialized")
        self.get_logger().info(f"  - Trigger topic: {self.trigger_topic}")
        self.get_logger().info(f"  - Object: '{self.object_name}' at {self.object_position} "
                               f"dims={self.object_dimensions}")
        self.get_logger().info(f"  - Hold: {self.attach_hold_seconds}s, remove_after={self.remove_after}")

    def _load_parameters(self):
        self.declare_parameter('add_service', '/curobo_trajectory_planner/add_object')
        self.declare_parameter('attach_service', '/curobo_trajectory_planner/attach_object')
        self.declare_parameter('detach_service', '/curobo_trajectory_planner/detach_object')
        self.declare_parameter('remove_service', '/curobo_trajectory_planner/remove_object')
        self.declare_parameter('trigger_topic', '/test/attach_detach')

        self.declare_parameter('object_name', 'test_attach_box')
        self.declare_parameter('object_position', [0.5, 0.0, 0.3])
        self.declare_parameter('object_dimensions', [0.1, 0.1, 0.1])
        self.declare_parameter('object_color', [0.2, 0.6, 1.0, 0.8])
        self.declare_parameter('attach_hold_seconds', 3.0)
        self.declare_parameter('remove_after', True)

        # Optional arm motion while the object is attached (so the green attached
        # spheres are seen riding the arm). Uses the unified "plan+execute" action.
        self.declare_parameter('move_enabled', True)
        self.declare_parameter('execute_action', '/curobo_trajectory_planner/execute_trajectory')
        self.declare_parameter('move_target_position', [0.4, 0.2, 0.5])
        self.declare_parameter('move_target_orientation', [1.0, 0.0, 0.0, 0.0])  # w,x,y,z

        self.add_service = self.get_parameter('add_service').value
        self.attach_service = self.get_parameter('attach_service').value
        self.detach_service = self.get_parameter('detach_service').value
        self.remove_service = self.get_parameter('remove_service').value
        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.object_name = self.get_parameter('object_name').value
        self.object_position = list(self.get_parameter('object_position').value)
        self.object_dimensions = list(self.get_parameter('object_dimensions').value)
        self.object_color = list(self.get_parameter('object_color').value)
        self.attach_hold_seconds = self.get_parameter('attach_hold_seconds').value
        self.remove_after = self.get_parameter('remove_after').value
        self.move_enabled = self.get_parameter('move_enabled').value
        self.execute_action = self.get_parameter('execute_action').value
        self.move_target_position = list(self.get_parameter('move_target_position').value)
        self.move_target_orientation = list(self.get_parameter('move_target_orientation').value)

    def _setup_interfaces(self):
        self.add_client = self.create_client(AddObject, self.add_service)
        self.attach_client = self.create_client(AttachObject, self.attach_service)
        self.detach_client = self.create_client(Trigger, self.detach_service)
        self.remove_client = self.create_client(RemoveObject, self.remove_service)
        self.execute_client = ActionClient(self, SendTrajectory, self.execute_action)

        self.trigger_sub = self.create_subscription(
            Bool, self.trigger_topic, self.on_trigger, 10
        )

    # ── Trigger / cycle ──────────────────────────────────────────────────────

    async def on_trigger(self, msg: Bool):
        if not msg.data or self._busy:
            return
        self._busy = True
        self.get_logger().info("Trigger received, starting attach/detach cycle")
        try:
            await self.run_cycle()
        finally:
            self._busy = False

    async def run_cycle(self):
        # 1. Add the cuboid obstacle to the scene.
        if not await self.add_object():
            self.get_logger().error("add_object failed, aborting cycle")
            return

        # 2. Attach it to the arm at the current configuration.
        if not await self.attach_object():
            self.get_logger().error("attach_object failed, aborting cycle")
            return

        # 3. Hold so the attached state is observable in RViz.
        self.get_logger().info(f"Attached — holding {self.attach_hold_seconds}s...")
        await self._sleep(self.attach_hold_seconds)

        # 4. Optionally move the arm while attached — the green attached spheres
        #    ride the wrist (the static grey obstacle marker stays put).
        if self.move_enabled:
            await self.execute_move()
            await self._sleep(self.attach_hold_seconds)

        # 5. Detach.
        await self.detach_object()

        # 5. Cleanup so a repeated trigger doesn't fail on a duplicate name.
        if self.remove_after:
            await self.remove_object()

        self.get_logger().info("Attach/detach cycle complete")

    # ── Service calls ────────────────────────────────────────────────────────

    async def add_object(self) -> bool:
        if not self.add_client.service_is_ready():
            self.get_logger().error(f"add service not available: {self.add_service}")
            return False
        req = AddObject.Request()
        req.type = AddObject.CUBOID
        req.name = self.object_name
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.object_position
        pose.orientation.w = 1.0
        req.pose = pose
        req.dimensions = Vector3(x=self.object_dimensions[0],
                                 y=self.object_dimensions[1],
                                 z=self.object_dimensions[2])
        req.color = ColorRGBA(r=self.object_color[0], g=self.object_color[1],
                              b=self.object_color[2], a=self.object_color[3])
        resp = await self.add_client.call_async(req)
        self._log_resp("add_object", resp)
        return resp.success

    async def attach_object(self) -> bool:
        if not self.attach_client.service_is_ready():
            self.get_logger().error(f"attach service not available: {self.attach_service}")
            return False
        req = AttachObject.Request()
        req.object_name = self.object_name
        resp = await self.attach_client.call_async(req)
        self._log_resp("attach_object", resp)
        return resp.success

    async def detach_object(self) -> bool:
        if not self.detach_client.service_is_ready():
            self.get_logger().error(f"detach service not available: {self.detach_service}")
            return False
        resp = await self.detach_client.call_async(Trigger.Request())
        self._log_resp("detach_object", resp)
        return resp.success

    async def remove_object(self) -> bool:
        if not self.remove_client.service_is_ready():
            self.get_logger().error(f"remove service not available: {self.remove_service}")
            return False
        req = RemoveObject.Request()
        req.name = self.object_name
        resp = await self.remove_client.call_async(req)
        self._log_resp("remove_object", resp)
        return resp.success

    async def execute_move(self) -> bool:
        """Move the arm to move_target_pose while the object is attached.

        NOTE: only the classic/open-loop planner actually carries the attached
        object in its collision model; MPC does not. The green attached-sphere
        visualization rides the arm regardless (it reads FK on the live joints).
        """
        if not self.execute_client.server_is_ready():
            self.get_logger().error(f"execute action not available: {self.execute_action}")
            return False
        goal = SendTrajectory.Goal()
        p = self.move_target_position
        q = self.move_target_orientation
        goal.target_pose.position.x, goal.target_pose.position.y, goal.target_pose.position.z = p
        goal.target_pose.orientation.w, goal.target_pose.orientation.x, \
            goal.target_pose.orientation.y, goal.target_pose.orientation.z = q
        goal.allow_cached = False
        self.get_logger().info(f"Moving (attached) to ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})...")
        goal_handle = await self.execute_client.send_goal_async(goal)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("move goal rejected")
            return False
        result = (await goal_handle.get_result_async()).result
        level = self.get_logger().info if result.success else self.get_logger().error
        level(f"move: success={result.success} msg='{result.message}'")
        return result.success

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _log_resp(self, label, resp):
        level = self.get_logger().info if resp.success else self.get_logger().error
        level(f"{label}: success={resp.success} msg='{resp.message}'")

    async def _sleep(self, seconds):
        """Non-blocking async sleep: a one-shot timer completes a Future we await
        (keeps the executor spinning; no blocking sleep, no re-spin)."""
        fut = rclpy.task.Future()

        def _done():
            timer.cancel()
            if not fut.done():
                fut.set_result(None)

        timer = self.create_timer(seconds, _done)
        await fut


def main(args=None):
    rclpy.init(args=args)
    node = AttachDetachTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
