#!/usr/bin/env python3
"""Where to score reachability: the workspace region, resolved from live ROS data.

What Q is scored over: the union of spheres around the MPC goal and the last few
poses of the MPC predicted path — the approach corridor the arm actually
traverses. The bare fiducial marker is *not* that target: grasp.py offsets the
tag by `approach_distance` (-0.2 m by default) along the marker Z to build the
pose it plans to, so scoring the tag alone optimises around a point the arm never
visits. The marker remains the fallback when no MPC goal is available.

Every center is transformed into the frame the voxel grid is published in, so
the region and the reachability map always live in the same space.
"""

from geometry_msgs.msg import Pose, PoseStamped
import math
from nav_msgs.msg import Path
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from ros2_markertracker_interfaces.msg import FiducialMarkerArray
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose_stamped
import tf2_ros
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from capacitynet.control.scripts.grid_markers import build_region_markers
from capacitynet.control.components.node_component import NodeComponent


class WorkspaceRegionSource(NodeComponent):
    """Resolves the scored workspace region from MPC goal / path / fiducial marker.

    Owns everything about *where* to score: the topics, the TF lookups, the
    staleness policy, the radius, and the debug spheres. A node that only wants
    to look at the region — no inference, no motion — instantiates just this.
    """

    def __init__(self, node, prefix='', callback_group=None, tf_buffer=None):
        """Attach the region source to `node` and subscribe to its inputs.

        Args:
            node: host rclpy Node
            prefix: optional parameter/topic prefix ('' keeps flat names)
            callback_group: callback group for the subscriptions
            tf_buffer: reuse an existing tf2 Buffer instead of creating one
        """
        super().__init__(node, prefix, callback_group)

        self.marker_topic = self._declare('marker_topic', '/fiducial_markers')
        self.target_marker_id = int(self._declare('target_marker_id', -1))
        # grasp.py republishes the offset grasp target on goal_topic at 10 Hz while
        # approaching, and the planner publishes the predicted path; together they
        # describe the corridor to score.
        self.goal_topic = self._declare(
            'goal_topic', '/curobo_trajectory_planner/mpc_goal')
        self.path_topic = self._declare('path_topic', '/mpc_predicted_path')
        # How many poses from the END of the path join the goal in the union. The
        # tail is anchored near the goal by the tag geometry, so it stays stable
        # across replans, unlike the full trajectory. 0 -> goal sphere only.
        self.path_tail_samples = int(self._declare('path_tail_samples', 4))
        # goal_topic carries a bare Pose with no header; assume this frame
        # (grasp.py always publishes it in the frame it passes to the action).
        self.planning_frame = self._declare('planning_frame', 'dsr01/base_link')
        self.marker_timeout = float(self._declare('marker_timeout', 2.0))
        self.mpc_timeout = float(self._declare('mpc_timeout', 2.0))
        self.workspace_radius = float(self._declare('workspace_radius', 0.10))
        # Radius of the path-tail spheres (region centers 1+), separate from
        # workspace_radius which sizes only the goal sphere (center 0). Defaults
        # to the same value so unconfigured setups score exactly as before this
        # param existed.
        self.path_radius = float(self._declare('path_radius', 0.05))
        # TF frame rigidly attached to the arm's TCP/end-effector. Empty (default)
        # disables the arm-in-workspace interlock entirely — there is no safe
        # guess for this across arm/gripper configs, and a wrong guess would
        # silently stop (or fail to stop) the base at the wrong moment. Verify
        # the real frame name against the live TF tree (`ros2 run tf2_tools
        # view_frames`) before setting it.
        self.ee_frame = self._declare('ee_frame', '')
        # Hysteresis band added to workspace_radius before the interlock releases,
        # so a TCP sitting on the boundary doesn't chatter the base on and off.
        self.arm_stop_margin = float(self._declare('arm_stop_margin', 0.05))
        # Fallback region center, expressed in the voxel grid frame. Needed for bag
        # replay: no recorded bag contains /fiducial_markers. Kept as a string
        # ("x,y,z" or "[x, y, z]") so it passes identically through a launch
        # argument and a --ros-args -p override.
        self.static_workspace_center = self._parse_center(
            self._declare('static_workspace_center', ''))
        self.publish_debug = bool(self._declare('publish_debug_markers', True))

        # Latest fiducial marker pose (in its own camera frame), or None.
        self._marker_pose = None
        self._marker_stamp = None

        # Latest MPC goal (planning_frame) and predicted-path tail (own frame).
        self._mpc_goal = None
        self._mpc_goal_stamp = None
        self._path_tail = None
        self._path_frame = None
        self._path_stamp = None

        # Which source produced the last region; logged when it changes, because
        # switching source changes what Q measures and so rescales the gradient.
        self._source = None
        self._n_region_markers = 0

        # Last resolved region, cached so arm_inside_workspace() can be polled by
        # BaseCommander's own timer (10 Hz) without waiting on the next inference
        # cycle (~2 Hz) that resolve() is normally driven by.
        self._last_centers = None
        self._last_frame = None
        self._arm_inside = False

        # Called when a region that *was* available goes stale. A controlling node
        # wires this to its stop command; a probe leaves it unset.
        self.on_lost = None

        self.tf_buffer = tf_buffer if tf_buffer is not None else Buffer()
        self._tf_listener = (
            None if tf_buffer is not None
            else TransformListener(self.tf_buffer, node))

        self._setup_interfaces()

    # ── Setup ────────────────────────────────────────────────────────────────

    def _setup_interfaces(self):
        node = self.node

        self.marker_sub = node.create_subscription(
            FiducialMarkerArray, self.marker_topic, self.on_markers, 10,
            callback_group=self.callback_group)

        # Only the newest goal/path matter — an older one describes a corridor the
        # arm has already left, so keep depth 1 rather than queueing.
        if self.goal_topic:
            self.goal_sub = node.create_subscription(
                Pose, self.goal_topic, self.on_mpc_goal, 1,
                callback_group=self.callback_group)
        if self.path_topic and self.path_tail_samples > 0:
            path_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.path_sub = node.create_subscription(
                Path, self.path_topic, self.on_path, path_qos,
                callback_group=self.callback_group)

        self.marker_pub = None
        self.source_pub = None
        if self.publish_debug:
            self.marker_pub = node.create_publisher(
                MarkerArray, self._private('region_markers'), 10)
            # Latched: a tool subscribing mid-run still learns what Q covers.
            source_qos = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.source_pub = node.create_publisher(
                String, self._private('region_source'), source_qos)

    @staticmethod
    def _parse_center(raw):
        """Parse a 'x,y,z' / '[x, y, z]' workspace center. Empty string -> None."""
        if not raw or not str(raw).strip():
            return None
        parts = [p for p in str(raw).replace('[', ' ').replace(']', ' ')
                 .replace(',', ' ').split() if p]
        if len(parts) != 3:
            raise ValueError(
                f'static_workspace_center must have 3 components, got {raw!r}')
        return tuple(float(p) for p in parts)

    # ── Public API ───────────────────────────────────────────────────────────

    @property
    def radius(self):
        """Region sphere radius in meters. Single source of truth for the package."""
        return self.workspace_radius

    @property
    def source(self):
        """Which input produced the last region: 'static' | 'mpc_goal+Npath' | 'marker'."""
        return self._source

    def resolve(self, target_frame):
        """Workspace region in `target_frame` as a list of centers, or None.

        Priority: the explicit static override, then the MPC goal together with
        the tail of the predicted path, then the fiducial marker. Q is the mean
        over the *union* of the spheres, so several centers describe the approach
        corridor rather than a single point.

        Also publishes the debug spheres and the region source, so a probe node
        writes no visualization code of its own.
        """
        centers = self._resolve_centers(target_frame)
        if centers is not None:
            self._last_centers = centers
            self._last_frame = target_frame
            self._publish_debug(centers, target_frame)
        return centers

    def arm_inside_workspace(self):
        """Whether the arm's TCP (`ee_frame`) is currently inside the scored region.

        Consulted by BaseCommander as a motion interlock, separate from the
        gradient-based convergence deadband: this stops the base the moment the
        arm has *physically* reached the workspace, regardless of what the
        gradient still says, on the theory that positioning is done once the arm
        can work there.

        Checked against each sphere at its own radius — workspace_radius for the
        goal (center 0), path_radius for the path tail — so a TCP entering
        either the goal sphere or the corridor counts as inside. Hysteresis: the
        interlock engages the instant any sphere is entered and only releases
        once the TCP clears every sphere by `arm_stop_margin`, so a TCP sitting
        on a boundary doesn't chatter the base on and off.

        Returns:
            bool: always False when `ee_frame` is unset (feature disabled) or no
                region has been resolved yet. On a TF lookup failure, holds the
                last known state rather than guessing.
        """
        if not self.ee_frame or self._last_centers is None:
            return False

        try:
            tf = self.tf_buffer.lookup_transform(
                self._last_frame, self.ee_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as ex:
            self.log.warn(
                f'TF {self.ee_frame} -> {self._last_frame} unavailable: {ex}',
                throttle_duration_sec=5.0)
            return self._arm_inside

        p = tf.transform.translation
        pt = (p.x, p.y, p.z)
        radii = self._radii_for(self._last_centers)
        min_margin = min(math.dist(pt, c) - r for c, r in zip(self._last_centers, radii))
        enter_threshold = self.arm_stop_margin if self._arm_inside else 0.0
        self._arm_inside = min_margin <= enter_threshold
        return self._arm_inside

    def _radii_for(self, centers):
        """Per-center sphere radii: workspace_radius for the goal (index 0),
        path_radius for every path-tail center.
        """
        return [self.workspace_radius] + [self.path_radius] * (len(centers) - 1)

    def fits(self, centers, vg_info, extra_margin=0.0):
        """Check that every region sphere stays inside the grid, with `extra_margin` in x/y.

        VoxelMask.compute_mean returns 0.0 for an empty mask, so a sphere that
        leaves the grid scores as *zero reachability* rather than *unknown* and
        would fabricate a large gradient pointing away from the boundary. The
        scored centers are `center - offset`, so with an NxN candidate grid they
        range over +/- (grid_size//2)*delta in x and y — that is what
        `extra_margin` covers (the caller passes `delta * (grid_size // 2)`).
        Every center is checked: a partially observed corridor would bias Q just
        as badly as a partially observed goal.
        """
        origin = vg_info['origin']
        res = vg_info['resolution']
        lo = (origin.x, origin.y, origin.z)
        hi = (origin.x + vg_info['size_x'] * res,
              origin.y + vg_info['size_y'] * res,
              origin.z + vg_info['size_z'] * res)
        radii = self._radii_for(centers)

        for idx, (center, radius) in enumerate(zip(centers, radii)):
            # Z is never offset by the candidate grid, only X and Y.
            margins = (radius + extra_margin, radius + extra_margin, radius)
            for axis, (c, m) in enumerate(zip(center, margins)):
                if c - m < lo[axis] or c + m > hi[axis]:
                    which = 'goal' if idx == 0 else f'path point {idx}'
                    self.log.warn(
                        f'Workspace sphere ({which}) leaves the voxel grid on axis '
                        f'{"xyz"[axis]} (center={c:+.3f}, margin={m:.3f}, '
                        f'bounds=[{lo[axis]:+.3f}, {hi[axis]:+.3f}]) — skipping cycle',
                        throttle_duration_sec=5.0)
                    return False
        return True

    # ── Callbacks ────────────────────────────────────────────────────────────

    def on_markers(self, msg: FiducialMarkerArray):
        """Store the pose of the tracked marker (in its own camera frame)."""
        if not msg.marker:
            return

        selected = None
        if self.target_marker_id < 0:
            selected = msg.marker[0]
        else:
            for m in msg.marker:
                if m.id == self.target_marker_id:
                    selected = m
                    break

        if selected is None:
            return

        pose = PoseStamped()
        pose.header = selected.pose_cov_stamped.header
        pose.pose = selected.pose_cov_stamped.pose.pose
        self._marker_pose = pose
        self._marker_stamp = self.node.get_clock().now()

    def on_mpc_goal(self, msg: Pose):
        """Store the grasp target grasp.py is driving the arm to."""
        self._mpc_goal = (msg.position.x, msg.position.y, msg.position.z)
        self._mpc_goal_stamp = self.node.get_clock().now()

    def on_path(self, msg: Path):
        """Store the tail of the predicted path, in the path's own frame."""
        if not msg.poses:
            return
        tail = msg.poses[-self.path_tail_samples:]
        self._path_tail = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                           for p in tail]
        self._path_frame = msg.header.frame_id or self.planning_frame
        self._path_stamp = self.node.get_clock().now()

    # ── Resolution ───────────────────────────────────────────────────────────

    def _resolve_centers(self, target_frame):
        if self.static_workspace_center is not None:
            self._note_source('static')
            return [self.static_workspace_center]

        centers = self._mpc_region(target_frame)
        if centers:
            self._note_source(f'mpc_goal+{len(centers) - 1}path')
            return centers

        centers = self._marker_region(target_frame)
        if centers:
            self._note_source('marker')
            return centers
        return None

    def _note_source(self, source):
        """Log region-source changes: they rescale Q, and so |grad Q|."""
        if source != self._source:
            if self._source is not None:
                self.log.info(
                    f'Workspace region source: {self._source} -> {source} '
                    '(Q now covers a different region, |grad Q| rescales)')
            self._source = source
            if self.source_pub is not None:
                self.source_pub.publish(String(data=source))

    def _mpc_region(self, target_frame):
        """Goal sphere plus the path tail, in target_frame. [] when unavailable."""
        if self._mpc_goal is None or self._age_s(self._mpc_goal_stamp) > self.mpc_timeout:
            return []

        centers = self._transform_positions(
            [self._mpc_goal], self.planning_frame, target_frame)
        if not centers:
            return []

        if self._path_tail and self._age_s(self._path_stamp) <= self.mpc_timeout:
            centers.extend(self._transform_positions(
                self._path_tail, self._path_frame, target_frame))
        return centers

    def _marker_region(self, target_frame):
        """Single sphere on the fiducial marker. [] when unavailable."""
        if self._marker_pose is None:
            self.log.warn(
                'No workspace target yet (no MPC goal, no marker), not commanding base',
                throttle_duration_sec=5.0)
            return []

        age = self._age_s(self._marker_stamp)
        if age > self.marker_timeout:
            self.log.warn(f'Marker stale ({age:.1f}s), not commanding base',
                          throttle_duration_sec=5.0)
            if self.on_lost is not None:
                self.on_lost()
            return []

        center = self._transform_marker_to(target_frame)
        return [center] if center is not None else []

    def _transform_positions(self, positions, source_frame, target_frame):
        """Transform (x, y, z) tuples between frames with one TF lookup. [] on failure."""
        if source_frame == target_frame:
            return list(positions)

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as ex:
            self.log.warn(
                f'TF {source_frame} -> {target_frame} unavailable: {ex}',
                throttle_duration_sec=5.0)
            return []

        transformed = []
        for x, y, z in positions:
            ps = PoseStamped()
            ps.header.frame_id = source_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            p = do_transform_pose_stamped(ps, tf).pose.position
            transformed.append((p.x, p.y, p.z))
        return transformed

    def _transform_marker_to(self, target_frame):
        """Transform the stored marker pose into target_frame. Returns (x,y,z) or None."""
        source_frame = self._marker_pose.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as ex:
            self.log.warn(
                f'TF {source_frame} -> {target_frame} unavailable: {ex}',
                throttle_duration_sec=5.0
            )
            return None

        transformed = do_transform_pose_stamped(self._marker_pose, tf)
        p = transformed.pose.position
        return (p.x, p.y, p.z)

    # ── Debug output ─────────────────────────────────────────────────────────

    def _publish_debug(self, centers, frame_id):
        if self.marker_pub is None:
            return
        array = build_region_markers(
            frame_id, self.node.get_clock().now().to_msg(),
            centers, self._radii_for(centers), n_stale=self._n_region_markers)
        self._n_region_markers = len(centers)
        self.marker_pub.publish(array)
