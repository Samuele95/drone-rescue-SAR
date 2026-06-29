"""Pixel-to-ground back-projection for the drones' nadir camera.

Pure-Python (no rclpy / cv2), so the geometry is unit-testable in isolation,
matching the ``lib/pid.py`` and ``lib/domain/navigation.py`` precedent.

Each drone carries a fixed downward-facing (nadir) camera
(``drone_rescue_gazebo/models/quadrotor/model.sdf``: camera ``pose`` pitched
+90deg). The image plane is therefore parallel to the ground: a detected
pixel's offset from the image centre is a bearing which, scaled by the drone's
altitude and **rotated by the drone's yaw**, yields the victim's world ground
position.

The original ``victim_detector._project_to_ground`` added the camera-frame
offset straight onto the drone position, ignoring yaw. Because the drones
continuously yaw while flying the search pattern, every off-centre detection
was rotated by the (ignored) heading, scattering confirmed victims tens of
metres from ground truth (every recorded run scored ``true_positives = 0``).
``project_pixel_to_ground`` reduces to the old formula exactly when
``drone_yaw == 0`` and is correct for every other heading.

Limitation: this assumes a pure-nadir camera. An off-nadir (tilted) camera
would need a full pinhole ray / ground-plane intersection instead.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple


def project_pixel_to_ground(
    pixel: Tuple[float, float],
    image_shape: Sequence[int],
    drone_x: float,
    drone_y: float,
    drone_z: float,
    drone_yaw: float,
    fov_rad: float,
) -> Tuple[float, float]:
    """Back-project an image pixel to a world ``(x, y)`` ground point.

    Args:
        pixel: ``(cx, cy)`` detected pixel, origin at the image's top-left.
        image_shape: ``(height, width, ...)``, e.g. ``ndarray.shape``.
        drone_x, drone_y, drone_z: drone world position (metres).
        drone_yaw: drone heading (radians); 0 = facing world +x.
        fov_rad: camera horizontal field of view (radians).

    Returns:
        ``(x, y)`` world ground coordinates (metres, z assumed 0).
    """
    img_h = float(image_shape[0])
    img_w = float(image_shape[1])
    cx, cy = pixel

    # Normalised offset from the image centre, range [-0.5, 0.5].
    offset_x = (cx - img_w / 2.0) / img_w
    offset_y = (cy - img_h / 2.0) / img_h

    # Ground footprint covered by the camera at this altitude.
    ground_width = 2.0 * drone_z * math.tan(fov_rad / 2.0)
    ground_height = ground_width * (img_h / img_w)

    # Camera/body-frame ground displacement. Image +x is right (body +x);
    # image +y points down the image, which maps to body -y. This matches
    # the original (yaw-0-only) formula.
    dx_body = offset_x * ground_width
    dy_body = -offset_y * ground_height

    # Rotate the body-frame displacement into the world frame by drone yaw.
    # The nadir camera is rigidly mounted, so the footprint yaws with the
    # drone; this rotation is the fix for the scattered-detections bug.
    cos_y = math.cos(drone_yaw)
    sin_y = math.sin(drone_yaw)
    dx_world = cos_y * dx_body - sin_y * dy_body
    dy_world = sin_y * dx_body + cos_y * dy_body

    return (drone_x + dx_world, drone_y + dy_world)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw (rotation about world +z) from a quaternion.

    Same formula used by ``drone_controller.odom_callback``; kept here so the
    detector and any test can derive yaw without depending on a node.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
