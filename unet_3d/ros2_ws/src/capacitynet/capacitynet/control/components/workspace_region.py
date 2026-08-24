#!/usr/bin/env python3
"""Where to score reachability: the workspace region, resolved from live ROS data.

What Q is scored over: the union of spheres around the MPC goal and the point
of the MPC predicted path the end effector is expected to reach `path_horizon_s`
seconds from now — the approach corridor the arm actually traverses. The bare
fiducial marker is *not* that target: grasp.py offsets the
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
        # Single path center: the predicted end-effector position this many
        # seconds from now (not from the path's publish stamp), i.e. where the
        # arm will actually be when it reaches that part of the corridor. Each
        # pose in the path carries its own header.stamp; we pick the pose whose
        # stamp is closest to (now + path_horizon_s). If the horizon exceeds
        # the path's time span, the nearest-stamp search naturally clamps to
        # the path's last pose. Negative -> path center disabled (goal sphere
        # only, matching the old path_tail_samples=0 behavior).
        self.path_horizon_s = float(self._declare('path_horizon_s', 2.0))
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

        # Latest MPC goal (planning_frame) and predicted path (own frame): a
        # list of (stamp_seconds, (x, y, z)) samples, sorted by stamp, used to
        # look up the pose at path_horizon_s.
        self._mpc_goal = None
        self._mpc_goal_stamp = None
        self._path_samples = None
        self._path_frame = None
        self._path_stamp = None

        # Which source produced the last region; logged when it changes, because
        # switching source changes what Q measures and so rescales the gradient.
        self._source = None
        self._n_region_markers = 0
        # Whether the last resolved region's index 0 is a true anchor (mpc_goal
        # or marker — gets workspace_radius) or a path-tail point standing in
        # for it (gets path_radius like the rest) — see _resolve_centers and
        # _radii_for.
        self._region_has_goal = True

        # Frame of the last resolved region, so region_spheres_touching() can
        # be polled by BaseCommander's own timer (10 Hz) without waiting on
        # the next inference cycle (~2 Hz) that resolve() is normally driven
        # by. Only the frame *name* is cached — region_spheres_touching()
        # re-resolves the centers fresh on every call (see its docstring).
        self._last_frame = None
        self._spheres_touching = False

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
        if self.path_topic and self.path_horizon_s >= 0.0:
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
        """Which input produced the last region.

        One of 'static' | 'mpc_goal+Npath' | 'path_only+N' | 'marker'.
        """
        return self._source

    @property
    def region_has_goal(self):
        """Whether the last resolved region's index 0 is a true mpc_goal center.

        False when the region was built from the path-horizon point alone (no
        fresh mpc_goal) — callers that assign workspace_radius to index 0 (see
        _radii_for, and GradientBasedController's own copy of that logic)
        need this to avoid treating a path point as the goal.
        """
        return self._region_has_goal

    def resolve(self, target_frame):
        """Workspace region in `target_frame` as a list of centers, or None.

        Priority: the explicit static override, then the MPC goal together with
        the path-horizon point of the predicted path, then the fiducial marker.
        Q is the mean over the *union* of the spheres, so both centers together
        describe the approach corridor rather than a single point.

        Also publishes the debug spheres and the region source, so a probe node
        writes no visualization code of its own.
        """
        centers = self._resolve_centers(target_frame)
        if centers is not None:
            self._last_frame = target_frame
            self._publish_debug(centers, target_frame)
        return centers

    def region_spheres_touching(self):
        """Whether the goal sphere and the path-horizon sphere touch or overlap.

        Consulted by BaseCommander as a motion interlock, separate from the
        gradient-based convergence deadband. The arm's own *measured* TCP
        position was tried here first and rejected: it lags the MPC's plan by
        the controller's live tracking error, so it doesn't reflect "the plan
        is about to finish". The path-horizon point (the predicted pose
        `path_horizon_s` seconds out — see WorkspaceRegionSource's module
        docstring) converges onto the goal as the plan nears completion
        instead, so contact between the two spheres is read as that signal.

        Only meaningful with exactly a goal and a path point (2 centers).
        Re-resolves the region fresh on every call (via `_resolve_centers`)
        instead of reusing `resolve()`'s cache, since this is polled at 10 Hz
        by BaseCommander's watchdog while `resolve()`'s own cache only
        refreshes once per ~1 Hz inference cycle.

        Returns:
            bool: False if the region has never been resolved yet (no
                `target_frame` known — otherwise gate() would never let the
                first resolve() cycle run at all). True (stop, by precaution)
                if fewer than 2 centers are currently available — no goal, or
                no fresh path point. Otherwise True iff the sphere surfaces
                touch or overlap (center distance <= sum of radii).
        """
        if self._last_frame is None:
            return False

        centers = self._resolve_centers(self._last_frame)
        if centers is None or len(centers) < 2:
            touching = True
        else:
            radii = self._radii_for(centers)
            distance = math.dist(centers[0], centers[1])
            touching = distance <= (radii[0] + radii[1])

        if touching != self._spheres_touching:
            self.log.info(
                f'{"Contact" if touching else "Separation"}: goal/path-horizon '
                f'spheres {"touching" if touching else "separated"}'
                + ('' if centers is None or len(centers) < 2 else
                   f' (center distance={distance:.3f}m, '
                   f'sum of radii={radii[0] + radii[1]:.3f}m)'))
            self._spheres_touching = touching
        # info, not debug: a node-wide debug level also floods the log with
        # rcl/rclpy internals (e.g. "Subscription take succeeded"), so this
        # throttled trace has to be visible at the default level to be usable.
        if centers is not None and len(centers) >= 2:
            self.log.info(
                f'region_spheres_touching: distance={distance:.3f}m, '
                f'sum_radii={radii[0] + radii[1]:.3f}m, touching={touching}',
                throttle_duration_sec=1.0)
        return touching

    def _radii_for(self, centers):
        """Per-center sphere radii: workspace_radius for the goal (index 0),
        path_radius for every path-tail center. When the region has no true
        goal (path-tail-only — see region_has_goal), every center gets
        path_radius, including index 0.
        """
        if self._region_has_goal:
            return [self.workspace_radius] + [self.path_radius] * (len(centers) - 1)
        return [self.path_radius] * len(centers)

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
        """Store the predicted path's timestamped poses, in the path's own frame."""
        if not msg.poses:
            return
        self._path_samples = [
            (rclpy.time.Time.from_msg(p.header.stamp).nanoseconds / 1e9,
             (p.pose.position.x, p.pose.position.y, p.pose.position.z))
            for p in msg.poses
        ]
        self._path_frame = msg.header.frame_id or self.planning_frame
        self._path_stamp = self.node.get_clock().now()

    # ── Resolution ───────────────────────────────────────────────────────────

    def _resolve_centers(self, target_frame):
        """Priority: static > mpc_goal(+path) > marker(+path) > path alone.

        The path tail joins whichever anchor is actually available (goal or
        marker) rather than being tied to the goal specifically — this lets
        a path-only publisher (e.g. mock_arm_trajectory, which never
        publishes mpc_goal) still combine with a live marker detection
        instead of being shadowed by it. Path-only (no anchor at all) stays
        the last resort, below the marker, so a continuously-republished
        mock path can never starve a real marker detection on its own.
        """
        if self.static_workspace_center is not None:
            self._region_has_goal = True
            self._note_source('static')
            return [self.static_workspace_center]

        path_centers = self._path_horizon_centers(target_frame)

        goal_centers = self._goal_center(target_frame)
        if goal_centers:
            self._region_has_goal = True
            self._note_source(f'mpc_goal+{len(path_centers)}path')
            return goal_centers + path_centers

        marker_centers = self._marker_region(target_frame)
        if marker_centers:
            self._region_has_goal = True
            self._note_source(
                f'marker+{len(path_centers)}path' if path_centers else 'marker')
            return marker_centers + path_centers

        if path_centers:
            self._region_has_goal = False
            self._note_source(f'path_only+{len(path_centers)}')
            return path_centers

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

    def _goal_center(self, target_frame):
        """Single mpc_goal sphere, in target_frame. [] when absent/stale."""
        goal_fresh = (self._mpc_goal is not None
                      and self._age_s(self._mpc_goal_stamp) <= self.mpc_timeout)
        if not goal_fresh:
            return []
        return self._transform_positions(
            [self._mpc_goal], self.planning_frame, target_frame)

    def _path_horizon_centers(self, target_frame):
        """Single path-horizon sphere, in target_frame. [] when absent/stale.

        Independent of mpc_goal freshness: this is a standalone anchor check
        so the path can join whichever anchor (goal or marker) is actually
        available — see _resolve_centers.
        """
        path_fresh = bool(self._path_samples) and self._age_s(self._path_stamp) <= self.mpc_timeout
        if not path_fresh or self.path_horizon_s < 0.0:
            return []
        target_time = self.node.get_clock().now().nanoseconds / 1e9 + self.path_horizon_s
        _, position = min(self._path_samples, key=lambda s: abs(s[0] - target_time))
        return self._transform_positions(
            [position], self._path_frame, target_frame)

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
