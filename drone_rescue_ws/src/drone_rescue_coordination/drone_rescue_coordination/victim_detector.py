#!/usr/bin/env python3
"""
Victim Detector Node

Subscribes to drone camera images and detects victims using:
1. ArUco marker detection (high confidence - for simulation)
2. Color-based detection (orange/red clothing)

Publishes detected victims for coverage tracking and visualization.
"""

import math
import threading
from typing import Dict, List, Optional
from collections import deque

from dataclasses import dataclass

import cv2
import numpy as np
from cv_bridge import CvBridge

from drone_rescue_coordination.lib.domain.elevation import ElevationModel
from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
from drone_rescue_coordination.lib.projection import (
    project_pixel_to_ground, yaw_from_quaternion,
)
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from drone_rescue_msgs.msg import VictimDetection


# Replaces an untyped dict that was carried in a flat List[dict]. Typed
# fields + a readable __repr__ make merge logic debuggable and prevent
# silent typos on attribute names.
@dataclass
class _VictimEstimate:
    id: int
    position: Point
    confidence: float
    detections: int = 1
    confirmed: bool = False
    type: int = 0          # mirrors VictimDetection.detection_type
    aruco_id: int = 0

    def __repr__(self) -> str:
        return (
            f'_VictimEstimate(#{self.id} '
            f'pos=({self.position.x:.1f},{self.position.y:.1f}) '
            f'conf={self.confidence:.2f} '
            f'detections={self.detections} '
            f'{"CONF" if self.confirmed else "tent"})'
        )


@dataclass
class _DronePose:
    """Drone pose used for pixel-to-ground projection.

    Carries yaw alongside position, the original code stored only a bare
    ``Point`` and discarded the orientation quaternion, which is exactly the
    omission that scattered every victim detection (see ``lib/projection.py``).
    Exposes ``.x/.y/.z`` so it is a drop-in for the previous ``Point`` stores.
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


class VictimDetector(Node):
    """
    Vision-based victim detection node.

    Subscribes:
        /{drone}/camera: RGB camera images
        /{drone}/odom: Drone position for victim localization

    Publishes:
        /victims/detected: PoseStamped for each new detection
        /victims/detections: Full VictimDetection messages
        /victims/count: Total victim count
    """

    def __init__(self):
        super().__init__('victim_detector')

        # Parameters
        self.declare_parameter('drone_names', default_drone_names_list())
        self.declare_parameter('detection_rate', 2.0)  # Hz
        self.declare_parameter('min_detection_height', 5.0)  # meters
        self.declare_parameter('max_detection_height', 25.0)
        self.declare_parameter('aruco_dict_id', 0)  # DICT_4X4_50
        self.declare_parameter('detection_threshold', 0.6)
        self.declare_parameter('victim_merge_radius', 3.0)  # meters
        self.declare_parameter('min_contour_area', 150)  # pixels
        self.declare_parameter('max_contour_area', 8000)  # pixels
        # 90deg matches the camera <horizontal_fov>1.5708</horizontal_fov> in
        # drone_rescue_gazebo/models/quadrotor/model.sdf. The old 60deg default
        # scaled every projected offset by tan(45)/tan(30) ~= 1.73x.
        self.declare_parameter('camera_fov_horizontal', 90.0)  # degrees
        self.declare_parameter('debug_detection', False)
        # Reject any candidate within this radius of any drone; colored drone
        # bodies show up in the HSV/orange color band and would otherwise produce
        # a flood of false positives at spawn.
        self.declare_parameter('min_distance_from_drones', 5.0)  # meters
        # The projected-area confidence above doesn't account for how far the
        # victim actually is from the drone. At survey altitude (25 m) + 90° FOV
        # the ground footprint is 50 m wide, so a victim at the edge gets the
        # same area-conf score as one directly below, making the camera behave
        # like a perfect long-range sensor. These two parameters introduce a
        # horizontal-range gate so detection quality decays with how close the
        # drone is to the target XY:
        #   xy <= range_decay_start   → no decay (full area_conf)
        #   start < xy < max_range    → linear decay to 0
        #   xy >= max_range           → drop the detection entirely
        # Default (5 / 12 m) means a drone must overfly within ~12 m of the
        # victim to even see it as a candidate, and orbit within ~5 m to score
        # high confidence, forcing the saga's INVESTIGATE → CONFIRM dance to
        # actually depend on physical proximity.
        self.declare_parameter('max_detection_range_m', 12.0)
        self.declare_parameter('range_decay_start_m', 5.0)
        # Terrain elevation gradient (m per m), mirroring mission_manager's
        # ``terrain_slope_x/y`` params. The detector must measure AGL, not
        # absolute odometry z: scan waypoints fly at constant above-ground
        # level (``survey_altitude + elevation_at(x, y)``,
        # mission_manager.py:1205-1211), so over terrain with elevation > 0 an
        # AGL-held drone's absolute z exceeds ``max_detection_height`` and the
        # old absolute-z gate silently disabled detection. Both 0.0
        # (default) = flat terrain = AGL identical to absolute z.
        self.declare_parameter('terrain_slope_x', 0.0)
        self.declare_parameter('terrain_slope_y', 0.0)
        self.declare_parameter('terrain_base', 0.0)

        self.drone_names = self.get_parameter('drone_names').value
        self.detection_rate = self.get_parameter('detection_rate').value
        self.min_height = self.get_parameter('min_detection_height').value
        self.max_height = self.get_parameter('max_detection_height').value
        self.aruco_dict_id = self.get_parameter('aruco_dict_id').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.merge_radius = self.get_parameter('victim_merge_radius').value
        self.min_area = self.get_parameter('min_contour_area').value
        self.max_area = self.get_parameter('max_contour_area').value
        self.camera_fov = self.get_parameter('camera_fov_horizontal').value
        self.debug_detection = self.get_parameter('debug_detection').value
        self.min_distance_from_drones = float(
            self.get_parameter('min_distance_from_drones').value
        )
        # Cache the two decay knobs: read once here, applied per-detection in
        # ``_apply_range_decay``. Re-read by the runtime callback below so
        # scenario YAML / Mission Control can adjust the range without
        # restarting the node.
        self.max_detection_range_m = float(
            self.get_parameter('max_detection_range_m').value
        )
        self.range_decay_start_m = float(
            self.get_parameter('range_decay_start_m').value
        )
        # Terrain model used to convert absolute odometry z to AGL before the
        # altitude gate. Built from the same param names mission_manager uses,
        # so a launch that plumbs terrain slope to both nodes keeps them in
        # agreement (see ``_within_detection_band``).
        self._elevation = ElevationModel.from_slopes(
            slope_x=float(self.get_parameter('terrain_slope_x').value),
            slope_y=float(self.get_parameter('terrain_slope_y').value),
            base=float(self.get_parameter('terrain_base').value),
        )

        # State
        self.bridge = CvBridge()
        self.drone_positions: Dict[str, _DronePose] = {}
        self.latest_images: Dict[str, Image] = {}
        # id → estimate. dict-by-id makes id lookups trivial; .values()
        # is iterated for the proximity scan in _process_detection.
        self.detected_victims: Dict[int, _VictimEstimate] = {}
        self.victim_counter = 0
        self.detection_stats: Dict[int, int] = {}  # aruco_id -> detection_count
        # Per-drone detection timers run concurrently under a
        # MultiThreadedExecutor; this lock serialises the shared victim-merge
        # state (detected_victims / victim_counter / detection_stats).
        self._victim_lock = threading.Lock()

        # ArUco setup. OpenCV 4.7+ uses DetectorParameters() to construct an
        # initialized object; on 4.6 (the apt ros-jazzy default) the same call
        # returns an uninitialized struct that segfaults on attribute set, so we
        # use the legacy factory there.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Increase adaptive threshold range for shadow handling
        # Default winSizeMax=23 is too small for Ogre2 shadows
        self.aruco_params.adaptiveThreshWinSizeMin = 5      # Default: 3
        self.aruco_params.adaptiveThreshWinSizeMax = 35     # Default: 23
        self.aruco_params.adaptiveThreshWinSizeStep = 5     # Default: 10

        # Relax contour filtering for markers partially in shadow/smoke
        self.aruco_params.minMarkerPerimeterRate = 0.01     # Default: 0.03
        self.aruco_params.polygonalApproxAccuracyRate = 0.05  # Default: 0.03

        # Improve bit extraction under visual noise (smoke/fire particles)
        self.aruco_params.perspectiveRemovePixelPerCell = 8   # Default: 4
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.15  # Default: 0.13

        # Enable sub-pixel corner refinement for better accuracy at altitude
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 7

        # Prevent false positives on uniform smoke/dust regions
        self.aruco_params.minOtsuStdDev = 5.0               # Default: 5.0

        # OpenCV 4.7+ exposes ArucoDetector; on 4.6 (the apt ros-jazzy default)
        # we use the module-level cv2.aruco.detectMarkers fallback in _detect_aruco.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_detector = None

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # TopicFactory replaces three hand-rolled per-drone patterns (raw_pubs,
        # camera subs, odom subs). The sensor subs override to SENSOR_HOT
        # (depth=1) so backlogged frames don't pile up on slow callbacks,
        # matches the legacy `sensor_qos` used here. Adding a new per-drone
        # topic now: one row in TOPIC_REGISTRY.
        from drone_rescue_coordination.lib.ros_adapter.topic_factory import (
            QosName, TopicFactory,
        )
        self._topic_factory = TopicFactory(self, self.drone_names)
        # Publishers: per-drone raw stream. detection_filter does
        # multi-view confirmation downstream.
        self.raw_pubs = self._topic_factory.per_drone_pubs(
            'detections_raw', VictimDetection,
        )
        self._camera_subs = self._topic_factory.per_drone_subs(
            'camera', Image, self.image_callback,
            qos_override=QosName.SENSOR_HOT,
        )
        self._odom_subs = self._topic_factory.per_drone_subs(
            'odom', Odometry, self.odom_callback,
            qos_override=QosName.SENSOR_HOT,
        )

        # One detection timer PER DRONE, each in its own
        # ReentrantCallbackGroup, so the fleet's CV pipelines run concurrently
        # under a MultiThreadedExecutor (see main) instead of serially on one
        # timer. Each drone's frame conversion + ArUco/colour detection no
        # longer blocks every other drone's.
        self._detection_cb_groups: Dict[str, ReentrantCallbackGroup] = {}
        self.detection_timers = {}
        for d in self.drone_names:
            group = ReentrantCallbackGroup()
            self._detection_cb_groups[d] = group
            self.detection_timers[d] = self.create_timer(
                1.0 / self.detection_rate,
                lambda dn=d: self._detect_for_drone(dn),
                callback_group=group,
            )

        self.stats_timer = self.create_timer(30.0, self._log_detection_stats)

        self.get_logger().info(
            f'Victim detector started for {len(self.drone_names)} drones, '
            f'detection rate: {self.detection_rate} Hz'
        )

    def odom_callback(self, msg: Odometry, drone_name: str):
        """Store drone pose (position + yaw) for victim localization.

        Yaw is required by the pixel-to-ground projection; storing only the
        position (the previous behaviour) is the root cause of the
        scattered-detection bug.
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.drone_positions[drone_name] = _DronePose(
            x=p.x, y=p.y, z=p.z,
            yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def image_callback(self, msg: Image, drone_name: str):
        """Store latest image from drone."""
        self.latest_images[drone_name] = msg

    def _within_detection_band(self, pos: '_DronePose') -> bool:
        """True when the drone's ABOVE-GROUND-LEVEL height is in the detection
        band ``[min_detection_height, max_detection_height]`` (inclusive).

        AGL = absolute odometry z minus the terrain elevation under the drone's
        (x, y). On flat terrain (the default) this equals the absolute z, so the
        historical behaviour is preserved; over sloped terrain it compensates
        for the constant-AGL flight altitude (mission_manager.py:1205-1211),
        which is exactly what the absolute-z gate failed to do (SWRL S6
        ``ex:drone_terrain_agl_example``).
        """
        agl = pos.z - self._elevation.elevation_at(pos.x, pos.y)
        return self.min_height <= agl <= self.max_height

    def detection_callback(self):
        """Process every drone's latest frame (single-threaded convenience).

        The live node drives one ``_detect_for_drone`` timer PER DRONE under a
        MultiThreadedExecutor, so the fleet's CV runs concurrently; this
        whole-fleet loop is kept for direct callers and single-threaded tests."""
        for drone_name in self.drone_names:
            self._detect_for_drone(drone_name)

    def _detect_for_drone(self, drone_name: str) -> None:
        """Run the CV pipeline for one drone's latest frame.

        Each drone has its own timer + ReentrantCallbackGroup, so this runs
        concurrently across drones; only the victim-merge step is serialised
        (see ``_process_detection``)."""
        if drone_name not in self.latest_images:
            return
        if drone_name not in self.drone_positions:
            return

        pos = self.drone_positions[drone_name]

        # Only detect at appropriate altitude, measured ABOVE GROUND LEVEL,
        # not absolute odometry z. Scan waypoints fly at constant AGL, so over
        # raised terrain the absolute z is offset by the terrain height; gating
        # on absolute z silently disabled the camera over slope.
        if not self._within_detection_band(pos):
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_images[drone_name],
                desired_encoding='bgr8'
            )
        except Exception as e:
            # Throttle: a persistently bad encoding (e.g. a mis-bridged camera)
            # fires this every detection tick for every drone, flooding the log.
            # One warning every 5 s surfaces the fault without drowning the
            # console.
            self.get_logger().warn(
                f'Image conversion failed: {e}',
                throttle_duration_sec=5.0,
            )
            return

        detections = []

        # 1. ArUco detection (high priority)
        aruco_detections = self._detect_aruco(cv_image, pos, drone_name)
        detections.extend(aruco_detections)

        # 2. Color-based detection (lower priority)
        color_detections = self._detect_by_color(cv_image, pos, drone_name)
        detections.extend(color_detections)

        # Process and publish detections: the CV above ran concurrently per
        # drone; the victim merge mutates shared state, so serialise it.
        with self._victim_lock:
            for detection in detections:
                victim_id = self._process_detection(detection)
                if victim_id is not None:
                    detection.victim_id = victim_id
                    self._publish_detection(detection)

    def _detect_aruco(self, image: np.ndarray, drone_pos: '_DronePose',
                      drone_name: str) -> List[VictimDetection]:
        """Detect ArUco markers (victim identifiers in simulation)."""
        detections = []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if self.debug_detection:
            if ids is not None:
                self.get_logger().info(
                    f'[{drone_name}] ArUco detected: IDs={ids.flatten().tolist()}, '
                    f'altitude={drone_pos.z:.1f}m'
                )
            else:
                rejected_count = len(rejected) if rejected is not None else 0
                self.get_logger().debug(
                    f'[{drone_name}] No ArUco detected, {rejected_count} rejected candidates, '
                    f'altitude={drone_pos.z:.1f}m'
                )

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # Track detection stats (shared across the concurrent per-drone
                # timers, guard the increment).
                with self._victim_lock:
                    self.detection_stats[int(marker_id)] = (
                        self.detection_stats.get(int(marker_id), 0) + 1)

                center = corners[i][0].mean(axis=0)

                victim_pos = self._project_to_ground(
                    center, image.shape, drone_pos
                )

                # Compute marker pixel area (signed area via shoelace formula).
                # detection_filter uses this to weight ArUco confidence: a
                # 4-px-wide marker at long range is far less reliable than a
                # 30-px one, so we want graded confidence rather than the
                # historical hard-coded 0.95.
                pts = corners[i][0]
                x = pts[:, 0]; y = pts[:, 1]
                marker_area_px = 0.5 * abs(
                    x[0] * (y[1] - y[3]) +
                    x[1] * (y[2] - y[0]) +
                    x[2] * (y[3] - y[1]) +
                    x[3] * (y[0] - y[2])
                )
                # Map area → confidence: 0 px → 0.4, 100 px → ~0.85, 400+ → 0.99.
                area_conf = max(0.4, min(0.99, 0.4 + 0.005 * marker_area_px))

                # Decay the area-based confidence by horizontal distance to the
                # drone; ``None`` means the victim is beyond the effective range
                # and the detection must be discarded.
                ranged_conf = self._apply_range_decay(
                    area_conf, victim_pos, drone_pos,
                )
                if ranged_conf is None:
                    continue

                detection = VictimDetection()
                detection.header.stamp = self.get_clock().now().to_msg()
                detection.header.frame_id = 'world'
                detection.drone_name = drone_name
                detection.victim_id = 0  # Will be assigned later
                detection.position = victim_pos
                detection.confidence = ranged_conf
                detection.detection_type = VictimDetection.DETECTION_ARUCO
                detection.aruco_id = int(marker_id)
                detection.confirmed = False
                detection.priority = 2  # High priority

                detections.append(detection)

        return detections

    def _detect_by_color(self, image: np.ndarray, drone_pos: '_DronePose',
                         drone_name: str) -> List[VictimDetection]:
        """Detect victims by clothing color (orange/red)."""
        detections = []

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect bright orange (rescue worker/victim clothing).
        # Upper hue trimmed 25 -> 20: the victims' orange clothing is H<=15
        # (e.g. victim_1 H=6, victim_4 jacket H=13), while the landing-pad ring
        # (H=22) and the road centerline (H=25) fell inside the old band and got
        # confirmed as false victims at/through the map centre. 20 keeps every
        # victim with margin and drops both static map features.
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([20, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # Detect red (victim marker clothing)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | \
                   cv2.inRange(hsv, lower_red2, upper_red2)

        # Detect skin tones
        lower_skin = np.array([0, 20, 70])
        upper_skin = np.array([20, 255, 255])
        mask_skin = cv2.inRange(hsv, lower_skin, upper_skin)

        mask = mask_orange | mask_red

        # Morphological clean-up. The original MORPH_OPEN (erode → dilate) with
        # a 3×3 kernel eroded victim blobs that are only ~3-5 px across in the
        # 480×360 frame at 25 m survey altitude. Real victims (0.5×0.8 m torso)
        # project to ~5×8 px → MORPH_OPEN wiped them. We keep MORPH_CLOSE
        # (dilate → erode) which connects nearby pixels without eroding the
        # blob, then rely on min_contour_area (lowered in the launch from 40 to
        # 5) for noise rejection. False positives are still gated downstream by
        # detection_filter's multi-view confirmation.
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Check if skin tone nearby (increases confidence)
                    roi_size = 30
                    skin_roi = mask_skin[
                        max(0, cy-roi_size):min(image.shape[0], cy+roi_size),
                        max(0, cx-roi_size):min(image.shape[1], cx+roi_size)
                    ]
                    has_skin = np.sum(skin_roi > 0) > 100

                    victim_pos = self._project_to_ground(
                        (cx, cy), image.shape, drone_pos
                    )

                    area_confidence = min(area / 2000.0, 1.0)
                    base_confidence = 0.6 if has_skin else 0.5
                    confidence = min(base_confidence + area_confidence * 0.3, 0.85)

                    # Same horizontal-range decay as the ArUco path. The visual
                    # classifier is the main source of long-range false
                    # positives (terrain blobs trigger the orange/red HSV band
                    # from any distance), so range gating here
                    # disproportionately benefits the noisy path.
                    ranged_conf = self._apply_range_decay(
                        confidence, victim_pos, drone_pos,
                    )
                    if ranged_conf is None:
                        continue

                    detection = VictimDetection()
                    detection.header.stamp = self.get_clock().now().to_msg()
                    detection.header.frame_id = 'world'
                    detection.drone_name = drone_name
                    detection.victim_id = 0
                    detection.position = victim_pos
                    detection.confidence = ranged_conf
                    detection.detection_type = VictimDetection.DETECTION_VISUAL
                    detection.aruco_id = -1
                    detection.confirmed = False
                    detection.priority = 1  # Medium priority

                    detections.append(detection)

        return detections

    def _project_to_ground(self, pixel: tuple, image_shape: tuple,
                           drone_pos: '_DronePose') -> Point:
        """Project a detected pixel to a world ground position.

        Thin wrapper over ``lib.projection.project_pixel_to_ground``; that
        function applies the drone-yaw rotation the original code omitted.
        """
        gx, gy = project_pixel_to_ground(
            pixel=pixel,
            image_shape=image_shape,
            drone_x=drone_pos.x,
            drone_y=drone_pos.y,
            drone_z=drone_pos.z,
            drone_yaw=drone_pos.yaw,
            fov_rad=math.radians(self.camera_fov),
        )
        victim_pos = Point()
        victim_pos.x = gx
        victim_pos.y = gy
        victim_pos.z = 0.0  # Ground level
        return victim_pos

    def _apply_range_decay(
        self, base_confidence: float, victim_pos: Point,
        drone_pos: '_DronePose',
    ) -> Optional[float]:
        """Decay a detection's confidence based on the horizontal distance
        between the drone and the projected victim position. Returns the new
        confidence, or ``None`` when the detection should be dropped entirely
        (beyond the configured max range).

        Why horizontal-only: at the project's fixed survey altitude (~25 m),
        the 3D range to a victim is dominated by altitude regardless of where
        the drone is hovering. The interesting signal, "is the drone NEAR the
        target", is the XY offset. A drone directly overhead has xy=0 and gets
        full confidence; one at the edge of its 50 m footprint has xy=25 m and
        gets nothing.

        Shape:
          xy <= range_decay_start_m  → full confidence
          start < xy < max_range     → linear decay
          xy >= max_range            → drop (return None)
        """
        dx = victim_pos.x - drone_pos.x
        dy = victim_pos.y - drone_pos.y
        xy = (dx * dx + dy * dy) ** 0.5
        if xy >= self.max_detection_range_m:
            return None
        if xy <= self.range_decay_start_m:
            return float(base_confidence)
        decay_band = self.max_detection_range_m - self.range_decay_start_m
        if decay_band <= 0.0:
            # Degenerate config (start >= max): treat as a hard cliff at start
            # and let the caller see full confidence inside it.
            return float(base_confidence)
        range_factor = 1.0 - (xy - self.range_decay_start_m) / decay_band
        return float(base_confidence) * max(0.0, range_factor)

    def _process_detection(self, detection: VictimDetection) -> Optional[int]:
        """
        Process detection and merge with existing victims.
        Returns victim_id if this is a new or updated victim, None if duplicate.
        """
        pos = detection.position

        # Self-filter (drone-on-drone false positives) is now enforced in
        # detection_filter so it owns one tunable knob instead of two.

        # Check against existing victims (proximity scan over the dict values).
        for victim in self.detected_victims.values():
            dx = victim.position.x - pos.x
            dy = victim.position.y - pos.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < self.merge_radius:
                victim.detections += 1

                # Position via weighted average.
                weight = 1.0 / victim.detections
                victim.position.x = (1 - weight) * victim.position.x + weight * pos.x
                victim.position.y = (1 - weight) * victim.position.y + weight * pos.y

                # Confidence increases with multiple detections.
                victim.confidence = min(
                    victim.confidence + 0.1 * detection.confidence,
                    0.99,
                )

                if victim.detections >= 3:
                    victim.confirmed = True

                # Only re-publish if significant update
                if victim.detections % 5 == 0:
                    return victim.id
                return None

        self.victim_counter += 1
        new_victim = _VictimEstimate(
            id=self.victim_counter,
            position=pos,
            confidence=detection.confidence,
            detections=1,
            confirmed=False,
            type=detection.detection_type,
            aruco_id=detection.aruco_id,
        )
        self.detected_victims[new_victim.id] = new_victim

        self.get_logger().info(
            f'New victim #{self.victim_counter} detected by {detection.drone_name} '
            f'at ({pos.x:.1f}, {pos.y:.1f}) '
            f'confidence={detection.confidence:.2f} '
            f'type={"ArUco" if detection.detection_type == VictimDetection.DETECTION_ARUCO else "Visual"}'
        )

        return self.victim_counter

    def _publish_detection(self, detection: VictimDetection):
        """Publish a per-drone raw detection. detection_filter does the
        cross-drone clustering / Bayesian fusion / multi-view confirmation
        and emits VictimCandidate downstream."""
        pub = self.raw_pubs.get(detection.drone_name)
        if pub is None:
            return
        pub.publish(detection)

    def get_victim_count(self) -> int:
        """Return total number of unique victims detected."""
        return len(self.detected_victims)

    def get_confirmed_count(self) -> int:
        """Return number of confirmed victims."""
        return sum(1 for v in self.detected_victims.values() if v.confirmed)

    def _log_detection_stats(self):
        """Log detection statistics for validation."""
        # Snapshot the shared dicts under the lock: this timer runs on a
        # different thread from the per-drone detection timers.
        with self._victim_lock:
            stats = dict(self.detection_stats)
            n_victims = len(self.detected_victims)
        if stats:
            stats_str = ', '.join(
                f'ArUco#{k}: {v}x' for k, v in sorted(stats.items())
            )
            self.get_logger().info(f'Detection stats: {stats_str}')
            self.get_logger().info(
                f'Unique victims: {n_victims}, '
                f'Confirmed: {self.get_confirmed_count()}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetector()
    # MultiThreadedExecutor so the per-drone detection timers (each in its own
    # ReentrantCallbackGroup) run concurrently instead of serialising the whole
    # fleet's CV on one thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
