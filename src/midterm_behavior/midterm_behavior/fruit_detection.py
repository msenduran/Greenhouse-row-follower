"""Ripe (red) fruit detection with an HSV color mask. Pure OpenCV, no ROS, so it can be unit tested."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class HsvThresholds:
    # Red wraps around hue 0 in OpenCV's 0-179 range, so two hue bands are used
    hue_low_max: int = 10
    hue_high_min: int = 170
    sat_min: int = 120   # rejects brown planters (S ~95) and foliage (S ~110)
    val_min: int = 50


@dataclass
class Detection:
    cx: float           # centroid in pixels
    cy: float
    area: float         # contour area in pixels
    bbox: tuple         # x, y, w, h
    offset_x: float     # -1 (left edge) .. +1 (right edge)
    offset_y: float     # -1 (top edge) .. +1 (bottom edge)
    area_ratio: float   # fraction of the image covered by the fruit


def red_mask(bgr, thresholds=HsvThresholds()):
    """Binary mask (uint8, 0/255) of red pixels, with small noise removed."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    t = thresholds
    low = cv2.inRange(hsv, (0, t.sat_min, t.val_min), (t.hue_low_max, 255, 255))
    high = cv2.inRange(hsv, (t.hue_high_min, t.sat_min, t.val_min), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def detect_fruit(bgr, thresholds=HsvThresholds(), min_area=150.0):
    """Return (largest red blob as Detection or None, mask)."""
    mask = red_mask(bgr, thresholds)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None, mask
    m = cv2.moments(contour)
    cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    h, w = mask.shape
    return Detection(
        cx=cx, cy=cy, area=area, bbox=cv2.boundingRect(contour),
        offset_x=(cx - w / 2) / (w / 2),
        offset_y=(cy - h / 2) / (h / 2),
        area_ratio=area / (w * h),
    ), mask


def draw_debug(bgr, detection, mask):
    """Camera image with the detection drawn on it and the mask as an inset (top-right)."""
    out = bgr.copy()
    h, w = out.shape[:2]
    # Image center cross-hair
    cv2.line(out, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
    cv2.line(out, (0, h // 2), (w, h // 2), (255, 255, 255), 1)
    if detection is not None:
        x, y, bw, bh = detection.bbox
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
        cv2.circle(out, (int(detection.cx), int(detection.cy)), 5, (0, 255, 255), -1)
        label = f'RIPE FRUIT  offset {detection.offset_x:+.2f}  area {100 * detection.area_ratio:.1f}%'
        cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        cv2.putText(out, 'no ripe fruit', (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    # Mask inset
    inset = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), (w // 4, h // 4))
    out[0:h // 4, w - w // 4:w] = inset
    cv2.rectangle(out, (w - w // 4, 0), (w - 1, h // 4), (255, 255, 255), 1)
    return out
