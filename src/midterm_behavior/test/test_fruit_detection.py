import numpy as np
import cv2

from midterm_behavior.fruit_detection import detect_fruit

# Colors sampled from the Gazebo camera (BGR)
FOLIAGE = (47, 83, 47)
PLANTER = (52, 68, 83)
RIPE = (25, 25, 112)
UNRIPE = (30, 150, 70)


def scene(fruit_color=None, center=(320, 175), radius=50):
    img = np.zeros((480, 640, 3), np.uint8)
    img[:] = FOLIAGE
    img[360:] = PLANTER
    if fruit_color is not None:
        cv2.circle(img, center, radius, fruit_color, -1)
    return img


def test_ripe_fruit_detected_at_center():
    det, _ = detect_fruit(scene(RIPE))
    assert det is not None
    assert abs(det.offset_x) < 0.02
    assert det.offset_y < 0  # above the image center


def test_offset_sign_follows_position():
    left, _ = detect_fruit(scene(RIPE, center=(100, 240)))
    right, _ = detect_fruit(scene(RIPE, center=(540, 240)))
    assert left.offset_x < -0.5
    assert right.offset_x > 0.5


def test_unripe_fruit_and_background_ignored():
    assert detect_fruit(scene())[0] is None
    assert detect_fruit(scene(UNRIPE))[0] is None


def test_tiny_blob_ignored():
    assert detect_fruit(scene(RIPE, radius=4))[0] is None
