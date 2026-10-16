"""Geometry helpers for the row follower. Pure Python/NumPy, no ROS, so they can be unit tested."""

import math

import numpy as np


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def sector_points(ranges, angle_min, angle_increment, start, end, max_range):
    """Valid scan points with start <= angle <= end (rad) and range <= max_range, as an Nx2 xy array."""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(r))
    keep = (angles >= start) & (angles <= end) & np.isfinite(r) & (r <= max_range)
    return np.column_stack((r[keep] * np.cos(angles[keep]), r[keep] * np.sin(angles[keep])))


def sector_min(ranges, angle_min, angle_increment, start, end):
    """Smallest finite range in the sector, or inf."""
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(r))
    sel = r[(angles >= start) & (angles <= end) & np.isfinite(r)]
    return float(sel.min()) if sel.size else math.inf


def fit_row(points, min_points=10, max_residual=0.03):
    """Fit a line to the row points (sensor frame, x forward, y left).

    Returns (distance, angle) or None:
      distance: perpendicular distance from the sensor to the line
      angle:    direction of the line relative to the robot heading, in (-pi/2, pi/2].
                0 = row parallel to the robot; negative = row turns away to the right ahead,
                i.e. the robot is heading to the left of the row.
    Returns None when the points are not on one straight line (rms distance to the line >
    max_residual), e.g. when the row's end face is in view at the start or end of a row.
    """
    if len(points) < min_points:
        return None
    mean = points.mean(axis=0)
    # Principal direction of the points = row direction
    _, s, vt = np.linalg.svd(points - mean)
    if s[1] / math.sqrt(len(points)) > max_residual:
        return None
    dx, dy = vt[0]
    if dx < 0:
        dx, dy = -dx, -dy
    angle = math.atan2(dy, dx)
    # Distance from the origin to the line through `mean` with direction (dx, dy)
    distance = abs(mean[0] * dy - mean[1] * dx)
    return distance, angle
