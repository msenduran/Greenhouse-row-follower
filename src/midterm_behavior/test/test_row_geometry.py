import math

import numpy as np

from midterm_behavior.row_geometry import fit_row, sector_min, sector_points, wrap_angle

ANGLE_MIN = -math.pi
INC = 2 * math.pi / 360


def scan_of_right_wall(distance, heading=0.0):
    """360-ray scan of an infinite wall on the right, robot rotated by `heading` (rad, left positive)."""
    ranges = []
    for i in range(360):
        a = ANGLE_MIN + i * INC
        # Wall normal points to -y in the world; in the sensor frame it is rotated by -heading
        n = -math.pi / 2 - heading
        c = math.cos(a - n)
        ranges.append(distance / c if c > 1e-6 else math.inf)
    return ranges


def test_wrap_angle():
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(wrap_angle(-3 * math.pi / 2), math.pi / 2)


def test_fit_parallel_wall():
    pts = sector_points(scan_of_right_wall(0.45), ANGLE_MIN, INC, -135 * math.pi / 180, -45 * math.pi / 180, 1.2)
    distance, angle = fit_row(pts)
    assert abs(distance - 0.45) < 0.01
    assert abs(angle) < 0.01


def test_fit_heading_left_gives_negative_angle():
    # Robot turned 10 deg to the left of the row -> row appears rotated clockwise
    pts = sector_points(scan_of_right_wall(0.45, heading=math.radians(10)), ANGLE_MIN, INC,
                        -135 * math.pi / 180, -45 * math.pi / 180, 1.2)
    _, angle = fit_row(pts)
    assert abs(angle - math.radians(-10)) < 0.02


def test_no_row_returns_none():
    assert fit_row(np.empty((0, 2))) is None


def test_sector_min():
    ranges = [math.inf] * 360
    ranges[180] = 0.4   # straight ahead
    assert sector_min(ranges, ANGLE_MIN, INC, -0.35, 0.35) == 0.4
    assert sector_min(ranges, ANGLE_MIN, INC, 1.0, 2.0) == math.inf


def test_corner_is_rejected():
    # Row side face plus its end face (L shape), as seen at the end of a row
    side = np.column_stack((np.linspace(-0.6, 0.0, 30), np.full(30, -0.45)))
    end = np.column_stack((np.zeros(15), np.linspace(-0.45, -1.0, 15)))
    assert fit_row(np.vstack((side, end))) is None
    assert fit_row(side) is not None
