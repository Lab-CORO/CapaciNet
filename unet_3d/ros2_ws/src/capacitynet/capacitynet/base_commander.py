#!/usr/bin/env python3
"""Turn grad Q into a mobile-base velocity command, safely.

Everything that decides *motion* lives here: gain, saturation, the adaptive
speed cap, the taper, the convergence deadband, the enable interlock, the
grasper interlock, and the watchdog that republishes (or zeroes) /cmd_vel.

Safety model: the commander starts DISABLED unless `start_enabled` is set, and
it disables motion whenever `grasp.py` reports a state outside
`grasper_allowed_states`. A node that never instantiates this component cannot
command the base at all — that is the point of keeping it separate.

The default allowed set is `idle,act_move_pregrasp` — exactly the window in
which `grasp.py` republishes `mpc_goal` live and its trajectory action keeps
correcting for a moving target. From `act_open_gripper` onward the arm holds
its last commanded pose with no active correction, so base motion there would
silently drift the achieved alignment instead of being compensated.
"""

import math

from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .node_component import NodeComponent

ZERO_TWIST = Twist()


def gradient_to_velocity(grad_x, grad_y, gain, max_vel):
    """Proportional control with saturation: v = k · ∇Q, clipped to `max_vel`.

    The single implementation of that law in the package — the mock harness and
    brain_orchestrator use it too, so tuning the saturation cannot drift between
    the nodes that drive the base.

    Args:
        grad_x, grad_y: float, gradient components
        gain: float, proportional gain k
        max_vel: float, speed ceiling in m/s

    Returns:
        tuple: (vx, vy) velocity command in m/s
    """
    vx = gain * grad_x
    vy = gain * grad_y

    magnitude = math.hypot(vx, vy)
    if magnitude > max_vel:
        scale = max_vel / magnitude
        vx *= scale
        vy *= scale

    return vx, vy


class BaseCommander(NodeComponent):
    """Publishes /cmd_vel from a gradient, with interlocks and a watchdog."""

    def __init__(self, node, delta, prefix='', callback_group=None):
        """Attach the commander to `node` and start its watchdog.

        Args:
            node: host rclpy Node
            delta: float, candidate grid spacing in meters — bounds how far the
                base may travel between two gradient updates
            prefix: optional parameter/topic prefix ('' keeps flat names)
            callback_group: callback group for the interlocks and the watchdog.
                Keep it separate from the inference group, or /cmd_vel collapses
                to the inference rate (~2 Hz) and most base drivers time out
                below ~4 Hz.
        """
        super().__init__(node, prefix, callback_group)
        self._delta = delta

        self.cmd_vel_topic = self._declare('cmd_vel_topic', '/cmd_vel')
        # base_speed is the hardware-validated ceiling; the adaptive cap below can
        # only lower it, never raise it.
        self.base_speed = float(self._declare('base_speed', 0.005))
        self.step_fraction = float(self._declare('step_fraction', 0.5))
        self.min_speed = float(self._declare('min_speed', 0.002))
        self.control_gain = float(self._declare('control_gain', 1.0))
        self.gradient_taper_ref = float(self._declare('gradient_taper_ref', 0.0))
        self.publish_rate = float(self._declare('publish_rate', 10.0))
        self.command_timeout = float(self._declare('command_timeout', 1.0))
        self.start_enabled = bool(self._declare('start_enabled', False))
        self.grasper_state_topic = self._declare(
            'grasper_state_topic', '/object_grasper/state')
        # Comma-separated states that permit base motion. See the module
        # docstring for why the default stops at act_move_pregrasp.
        self.grasper_allowed_states = frozenset(
            s.strip() for s in
            str(self._declare('grasper_allowed_states', 'idle,act_move_pregrasp')).split(',')
            if s.strip())
        # Whether the arm has physically reached the scored workspace. A
        # controlling node wires this to WorkspaceRegionSource.arm_inside_workspace;
        # left alone, the interlock never engages (region.ee_frame is empty by
        # default anyway, so this is belt-and-suspenders for a probe/mock node
        # that never wires it at all).
        self.arm_inside_workspace = lambda: False

        self._enabled = self.start_enabled
        self._converged = False
        # None = never heard from the grasper. Treated as permissive so the
        # controller still runs standalone when grasp.py is not up.
        self._grasper_state = None
        # EWMA of the measured cycle time; None until the first cycle completes.
        self._cycle_ewma = None

        # Last computed command, republished by the watchdog until it goes stale.
        self._last_twist = Twist()
        self._last_cmd_time = None

        self._setup_interfaces()

    # ── Setup ────────────────────────────────────────────────────────────────

    def _setup_interfaces(self):
        node = self.node

        self.cmd_vel_pub = node.create_publisher(Twist, self.cmd_vel_topic, 10)

        if self.grasper_state_topic:
            # Transient-local to match grasp.py's publisher, otherwise the latched
            # current state is missed and only the next transition is seen.
            state_qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.grasper_state_sub = node.create_subscription(
                String, self.grasper_state_topic, self.on_grasper_state, state_qos,
                callback_group=self.callback_group)

        self.enable_sub = node.create_subscription(
            Bool, self._private('enable'), self.on_enable_msg, 10,
            callback_group=self.callback_group)
        self.enable_srv = node.create_service(
            SetBool, self._private('enable'), self.on_enable_srv,
            callback_group=self.callback_group)

        self.timer = node.create_timer(
            1.0 / self.publish_rate, self.publish_command,
            callback_group=self.callback_group)

    # ── Public API ───────────────────────────────────────────────────────────

    @property
    def enabled(self):
        """Whether the enable topic/service currently permits commanding."""
        return self._enabled

    def allows_motion(self):
        """Enabled, grasper idle, not converged, and the arm not already inside the workspace."""
        return (self._enabled and not self._converged and self._grasper_allows_motion()
                and not self.arm_inside_workspace())

    def submit(self, gradient, cycle_s, score_center=None):
        """Shape a gradient into the commanded velocity and cache it.

        Gain, then the adaptive cap, then the optional taper, then the
        convergence deadband.

        Args:
            gradient: (grad_x, grad_y) of Q, per meter
            cycle_s: how long the cycle that produced it took, in seconds
            score_center: Q at the current position, only used in the convergence log

        Returns:
            tuple: ((vx, vy) commanded in m/s, v_cap applied in m/s)
        """
        # The cap uses the *previous* cycles' EWMA, matching the fact that the
        # command it limits was computed over a cycle that has just ended.
        v_cap = self._velocity_cap()
        self._cycle_ewma = cycle_s if self._cycle_ewma is None else (
            0.3 * cycle_s + 0.7 * self._cycle_ewma)

        grad_x, grad_y = gradient
        vx, vy = gradient_to_velocity(grad_x, grad_y, self.control_gain, v_cap)

        # Optional quadratic taper: sharpens deceleration into the optimum. Only
        # active below g_ref, where saturation no longer binds, so it does not
        # slow the approach from far away.
        if self.gradient_taper_ref > 0.0:
            scale = min(1.0, math.hypot(grad_x, grad_y) / self.gradient_taper_ref)
            vx *= scale
            vy *= scale

        speed = math.hypot(vx, vy)
        if speed < self.min_speed:
            self._converged = True
            self.log.info(
                f'Converged: |grad Q|={math.hypot(grad_x, grad_y):.5f} gives '
                f'{speed:.4f} m/s < min_speed={self.min_speed} m/s. '
                + (f'Q_center={score_center:.4f}. ' if score_center is not None else '')
                + 'Re-enable to resume.')
            self._last_twist = Twist()
            self._last_cmd_time = self.node.get_clock().now()
            return (0.0, 0.0), v_cap

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        self._last_twist = twist
        self._last_cmd_time = self.node.get_clock().now()
        return (vx, vy), v_cap

    def stop(self):
        """Command an immediate stop and forget the cached velocity."""
        self._last_twist = Twist()
        self._last_cmd_time = None
        self.cmd_vel_pub.publish(ZERO_TWIST)

    def shutdown(self):
        """Best-effort stop, safe to call while the context is being torn down."""
        try:
            self._enabled = False
            self.stop()
        except Exception as ex:  # noqa: BLE001 - shutdown must never raise
            self.log.warn(f'Could not publish stop command: {ex}')

    # ── Watchdog ─────────────────────────────────────────────────────────────

    def publish_command(self):
        """Republish the last command, or zero when it cannot be trusted."""
        twist = self._last_twist

        if not self.allows_motion():
            twist = ZERO_TWIST
        elif self._last_cmd_time is None:
            twist = ZERO_TWIST
        else:
            age = self._age_s(self._last_cmd_time)
            if age > self.command_timeout:
                self.log.warn(
                    f'No reachability update for {age:.1f}s (> {self.command_timeout}s), '
                    'stopping base',
                    throttle_duration_sec=5.0)
                twist = ZERO_TWIST

        self.cmd_vel_pub.publish(twist)

    # ── Interlocks ───────────────────────────────────────────────────────────

    def _velocity_cap(self):
        """Speed ceiling keeping travel per cycle within step_fraction * delta."""
        if self._cycle_ewma is None or self._cycle_ewma <= 0.0:
            return self.base_speed
        cap = self.step_fraction * self._delta / self._cycle_ewma
        return min(self.base_speed, cap)

    def _grasper_allows_motion(self):
        # Unknown state (grasper not running) is permissive by design.
        return self._grasper_state is None or self._grasper_state in self.grasper_allowed_states

    def _set_enabled(self, value, source):
        value = bool(value)
        if value != self._enabled:
            self.log.info(f'{"Enabled" if value else "Disabled"} via {source}')
        self._enabled = value
        self._converged = False
        if not value:
            self.stop()

    def on_enable_msg(self, msg: Bool):
        self._set_enabled(msg.data, 'topic')

    def on_enable_srv(self, request, response):
        self._set_enabled(request.data, 'service')
        response.success = True
        response.message = 'enabled' if self._enabled else 'disabled'
        return response

    def on_grasper_state(self, msg: String):
        previous = self._grasper_state
        self._grasper_state = msg.data
        if previous == msg.data:
            return
        if msg.data not in self.grasper_allowed_states:
            self.log.info(f"Grasper busy ('{msg.data}') — holding base still")
            self.stop()
        elif previous is not None and previous not in self.grasper_allowed_states:
            self.log.info(f"Grasper state '{msg.data}' — base motion allowed again")
