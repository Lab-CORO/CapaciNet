#!/usr/bin/env python3
"""RViz MarkerArray builders for the reachability pipeline.

Pure functions: they take numbers and return a MarkerArray, so the component
that owns the data publishes it and nothing has to reach into another
component's state to draw it.
"""

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def build_grid_markers(frame_id, stamp, center, scores, gradient, offsets,
                       cell_size, floor_z=0.0, ns='reachability_grid'):
    """Draw the 9 candidate scores as a flat floor heatmap, plus the gradient arrow.

    One CUBE_LIST tiles the 3x3 candidate grid edge-to-edge at `floor_z`, colored
    per-cell by Q, so it reads as a small 2D map rather than 9 floating spheres at
    candidate-query height. Tile i sits at `center + offset_i` (in x/y only — the
    scored workspace center is `center - offset_i`; the sign is flipped here so
    the display reads as a motion suggestion rather than a workspace
    displacement), same in-plane position the spheres used to occupy.

    A single CUBE_LIST is one draw call regardless of cell count — cheap on a
    Jetson's GPU today at 9 cells, and it stays cheap if the grid ever gets
    denser, unlike one Marker per cell.

    Args:
        frame_id: str, frame the centers are expressed in
        stamp: builtin_interfaces/Time for every marker header
        center: (x, y, z) the grid is drawn around (the region's first center);
            only x/y are used, z is replaced by `floor_z`
        scores: sequence of 9 quality scores
        gradient: (grad_x, grad_y)
        offsets: 9 (offset_x, offset_y) tuples, in grid-index order
        cell_size: float, tile edge length in meters — pass the same grid
            spacing (delta) the offsets were built from, so tiles touch with no
            gaps or overlap
        floor_z: float, z height (in frame_id) to draw the flat map at
        ns: marker namespace

    Returns:
        visualization_msgs/MarkerArray: id 0 the heatmap, id 1 the arrow
    """
    lo, hi = float(min(scores)), float(max(scores))
    span = (hi - lo) or 1.0
    best = max(range(9), key=lambda i: float(scores[i]))

    array = MarkerArray()

    heat = Marker()
    heat.header.frame_id = frame_id
    heat.header.stamp = stamp
    heat.ns = ns
    heat.id = 0
    heat.type = Marker.CUBE_LIST
    heat.action = Marker.ADD
    heat.pose.orientation.w = 1.0
    heat.scale.x = heat.scale.y = cell_size
    heat.scale.z = 0.01
    for i in range(9):
        ox, oy = offsets[i]
        t = (float(scores[i]) - lo) / span
        heat.points.append(Point(x=center[0] + ox, y=center[1] + oy, z=floor_z))
        heat.colors.append(ColorRGBA(
            r=t, g=0.2, b=1.0 - t, a=1.0 if i == best else 0.7))
    array.markers.append(heat)

    gx, gy = gradient
    arrow = Marker()
    arrow.header.frame_id = frame_id
    arrow.header.stamp = stamp
    arrow.ns = ns
    arrow.id = 1
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose.orientation.w = 1.0
    norm = math.hypot(gx, gy) or 1.0
    arrow_z = floor_z + 0.02  # clears the heatmap surface, avoids z-fighting
    arrow.points = [
        Point(x=center[0], y=center[1], z=arrow_z),
        Point(x=center[0] + 0.15 * gx / norm,
              y=center[1] + 0.15 * gy / norm,
              z=arrow_z),
    ]
    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.01, 0.02, 0.02
    arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 0.0, 0.9
    array.markers.append(arrow)

    return array


def build_region_markers(frame_id, stamp, centers, radius, n_stale=0,
                         ns='workspace_region'):
    """Draw the scored region itself, at true radius — what Q averages over.

    Drawing it makes the union visible rather than inferred: the goal sphere is
    opaque-ish, the path tail fainter.

    Args:
        frame_id: str, frame the centers are expressed in
        stamp: builtin_interfaces/Time for every marker header
        centers: list of (x, y, z) region sphere centers; index 0 is the goal
        radius: float sphere radius shared by every sphere, or a sequence of
            floats matching centers (one radius per sphere — e.g. the goal
            drawn larger/smaller than the path tail)
        n_stale: how many spheres the previous region had, so leftovers are DELETEd
        ns: marker namespace

    Returns:
        visualization_msgs/MarkerArray
    """
    radii = radius if hasattr(radius, '__len__') else [radius] * len(centers)

    array = MarkerArray()

    for i, (c, r) in enumerate(zip(centers, radii)):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = c
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 2.0 * r
        m.color.r, m.color.g, m.color.b = (0.1, 0.9, 0.4) if i == 0 else (0.9, 0.6, 0.1)
        m.color.a = 0.12 if i == 0 else 0.07
        array.markers.append(m)

    # Drop spheres left over from a larger previous region.
    for i in range(len(centers), n_stale):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = i
        m.action = Marker.DELETE
        array.markers.append(m)

    return array
