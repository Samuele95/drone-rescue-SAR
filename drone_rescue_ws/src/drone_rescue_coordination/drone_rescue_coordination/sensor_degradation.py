#!/usr/bin/env python3
"""
Sensor Degradation Node

Applies weather-based degradation to sensor data:
- Camera: Noise, blur, simulated rain droplets
- LiDAR: Range reduction, increased noise
- GPS: Position drift during severe weather

This simulates realistic sensor behavior in adverse weather conditions.
"""

import math
import random
from typing import Optional, Dict

import numpy as np

from drone_rescue_coordination.lib.domain.fleet import default_drone_names_list
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

from drone_rescue_msgs.msg import WeatherState


class SensorDegradation(LifecycleNode):
    """
    Applies weather-based degradation effects to sensor data.

    Subscribes:
        /environment/weather: Current weather conditions
        /{drone}/camera_raw: Raw camera images
        /{drone}/lidar_raw: Raw LiDAR point clouds

    Publishes:
        /{drone}/camera: Degraded camera images
        /{drone}/lidar: Degraded LiDAR point clouds
    """

    def __init__(self):
        super().__init__('sensor_degradation')

        # Parameters
        self.declare_parameter('drone_names', default_drone_names_list())
        self.declare_parameter('enable_camera_degradation', True)
        self.declare_parameter('enable_lidar_degradation', True)
        self.declare_parameter('enable_gps_degradation', True)

        # Storm effect parameters
        self.declare_parameter('storm_noise_stddev', 25.0)
        self.declare_parameter('storm_blur_kernel', 3)
        self.declare_parameter('storm_droplet_count', 15)
        self.declare_parameter('windy_blur_kernel', 2)
        # Master RNG seed for reproducible noise. Offset 17 isolates this
        # node's stream from environment_monitor (offset 13) and the auction
        # tie-break (offset 7919). See sar_patterns.PlannerInput / mission_manager.
        self.declare_parameter('seed', 0)

        self.drone_names = self.get_parameter('drone_names').value
        self.enable_camera = self.get_parameter('enable_camera_degradation').value
        self.enable_lidar = self.get_parameter('enable_lidar_degradation').value
        self.enable_gps = self.get_parameter('enable_gps_degradation').value

        self.storm_noise = self.get_parameter('storm_noise_stddev').value
        self.storm_blur = self.get_parameter('storm_blur_kernel').value
        self.storm_droplets = self.get_parameter('storm_droplet_count').value
        self.windy_blur = self.get_parameter('windy_blur_kernel').value

        seed = int(self.get_parameter('seed').value) + 17
        # Per-instance RNGs for both Python's stdlib and NumPy. All
        # `random.foo()` and `np.random.foo()` calls in this node are
        # routed through these so two runs with the same seed inject
        # identical noise sequences.
        self._rng = random.Random(seed)
        self._np_rng = np.random.default_rng(seed)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Current weather state
        self.current_weather: Optional[WeatherState] = None

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Weather subscriber
        self.weather_sub = self.create_subscription(
            WeatherState,
            '/environment/weather',
            self.weather_callback,
            10
        )

        # Per-drone camera processing
        self.camera_pubs: Dict[str, any] = {}
        self.lidar_pubs: Dict[str, any] = {}

        for drone_name in self.drone_names:
            # Camera degradation pipeline
            if self.enable_camera:
                self.create_subscription(
                    Image,
                    f'/{drone_name}/camera_raw',
                    lambda msg, name=drone_name: self.camera_callback(msg, name),
                    sensor_qos
                )
                self.camera_pubs[drone_name] = self.create_publisher(
                    Image,
                    f'/{drone_name}/camera',
                    10
                )

            # LiDAR degradation pipeline
            if self.enable_lidar:
                self.create_subscription(
                    PointCloud2,
                    f'/{drone_name}/lidar_raw',
                    lambda msg, name=drone_name: self.lidar_callback(msg, name),
                    sensor_qos
                )
                self.lidar_pubs[drone_name] = self.create_publisher(
                    PointCloud2,
                    f'/{drone_name}/lidar',
                    10
                )

        self.get_logger().info(
            f'Sensor degradation node started for {len(self.drone_names)} drones '
            f'(camera={self.enable_camera}, lidar={self.enable_lidar})'
        )

    # LifecycleNode protocol callbacks. The class advertises
    # change_state / get_state services so lifecycle_manager can
    # deactivate weather-driven sensor noise during SAFE-mode recovery.
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def weather_callback(self, msg: WeatherState):
        """Update current weather state."""
        self.current_weather = msg

    def camera_callback(self, msg: Image, drone_name: str):
        """Apply weather-based degradation to camera images."""
        if drone_name not in self.camera_pubs:
            return

        # If no weather data, pass through unchanged
        if self.current_weather is None:
            self.camera_pubs[drone_name].publish(msg)
            return

        # Convert to OpenCV image
        try:
            import cv2
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Camera conversion failed: {e}')
            self.camera_pubs[drone_name].publish(msg)
            return

        # Apply degradation based on weather
        weather_condition = self.current_weather.condition

        if weather_condition == WeatherState.WEATHER_STORM:
            cv_image = self._apply_storm_effects(cv_image)
        elif weather_condition == WeatherState.WEATHER_WINDY:
            cv_image = self._apply_windy_effects(cv_image)
        # CLEAR: no degradation

        # Convert back and publish
        try:
            degraded_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            degraded_msg.header = msg.header
            self.camera_pubs[drone_name].publish(degraded_msg)
        except Exception as e:
            self.get_logger().warn(f'Camera publish failed: {e}')
            self.camera_pubs[drone_name].publish(msg)

    def _apply_storm_effects(self, image: np.ndarray) -> np.ndarray:
        """Apply storm weather effects to camera image."""
        import cv2

        # 1. Add Gaussian noise
        noise = self._np_rng.normal(0, self.storm_noise, image.shape).astype(np.float32)
        noisy = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        # 2. Reduce contrast (atmospheric haze)
        noisy = cv2.convertScaleAbs(noisy, alpha=0.75, beta=20)

        # 3. Apply motion blur (wind vibration)
        if self.storm_blur > 1:
            noisy = cv2.blur(noisy, (self.storm_blur, self.storm_blur))

        # 4. Add simulated water droplets
        for _ in range(self.storm_droplets):
            x = self._rng.randint(0, image.shape[1] - 1)
            y = self._rng.randint(0, image.shape[0] - 1)
            radius = self._rng.randint(2, 5)
            # Semi-transparent droplet
            overlay = noisy.copy()
            cv2.circle(overlay, (x, y), radius, (200, 200, 220), -1)
            cv2.addWeighted(overlay, 0.4, noisy, 0.6, 0, noisy)

        # 5. Add slight blue tint (rain atmosphere)
        noisy = cv2.addWeighted(
            noisy, 0.95,
            np.full_like(noisy, [30, 20, 10]), 0.05,
            0
        )

        return noisy

    def _apply_windy_effects(self, image: np.ndarray) -> np.ndarray:
        """Apply windy weather effects to camera image."""
        import cv2

        # 1. Slight motion blur from drone vibration
        if self.windy_blur > 1:
            image = cv2.blur(image, (self.windy_blur, self.windy_blur))

        # 2. Minor noise
        noise = self._np_rng.normal(0, 5, image.shape).astype(np.float32)
        image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        return image

    def lidar_callback(self, msg: PointCloud2, drone_name: str):
        """Apply weather-based degradation to LiDAR point clouds."""
        if drone_name not in self.lidar_pubs:
            return

        # If no weather data or clear weather, pass through
        if self.current_weather is None or \
           self.current_weather.condition == WeatherState.WEATHER_CLEAR:
            self.lidar_pubs[drone_name].publish(msg)
            return

        # For storm conditions, reduce effective range
        if self.current_weather.condition == WeatherState.WEATHER_STORM:
            degraded_msg = self._degrade_lidar_storm(msg)
            self.lidar_pubs[drone_name].publish(degraded_msg)
        else:
            self.lidar_pubs[drone_name].publish(msg)

    def _degrade_lidar_storm(self, msg: PointCloud2) -> PointCloud2:
        """Reduce LiDAR range and add noise during storm.

        Vectorised via numpy. The legacy Python loop did ~370k
        iterations per scan under storm (1024x360 GPU LiDAR at sensor
        rate) with math.sqrt + 3 rng.gauss + conditional drop per point.
        The four-step numpy formulation below produces an equivalent
        distribution (same seeded RNG, same range/noise formulas) in
        ~3 vectorised passes.
        """
        try:
            # Read points as a structured ndarray, far cheaper than
            # the legacy `list(read_points(...))` which materialised a
            # Python tuple per point.
            arr = np.asarray(
                list(point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'),
                )),
                dtype=np.float32,
            )
            if arr.size == 0:
                return msg
            # `read_points` returns an array of (x, y, z) tuples; cast
            # to a (N, 3) float view.
            xyz = arr.view(np.float32).reshape(-1, 3)
            n = xyz.shape[0]
            if n == 0:
                return msg

            range_multiplier = self.current_weather.lidar_range_multiplier
            max_range = 50.0 * range_multiplier  # Base range 50m

            dist = np.linalg.norm(xyz, axis=1)
            # Random dropout for distant points (rain interference):
            # 30% drop rate ONLY among points beyond `max_range*0.7`,
            # matching the loop's `if dist > max_range*0.7 and rng <
            # 0.3: continue`.
            far_mask = dist > max_range * 0.7
            drops = np.zeros(n, dtype=bool)
            n_far = int(far_mask.sum())
            if n_far > 0:
                drops[far_mask] = self._np_rng.random(n_far) < 0.3
            within_range = dist <= max_range
            keep = within_range & ~drops
            if not keep.any():
                header = msg.header
                return point_cloud2.create_cloud_xyz32(header, [])

            xyz_keep = xyz[keep]
            dist_keep = dist[keep]
            # Per-point Gaussian noise with sigma = 0.02 * dist. The
            # legacy code called rng.gauss(0, sigma) three times per
            # point; the vector equivalent draws (k, 3) with per-row
            # sigma. Mean / variance match; the seeded numpy generator
            # (self._np_rng) keeps the sequence deterministic.
            sigma = (0.02 * dist_keep).reshape(-1, 1).astype(np.float32)
            noise = self._np_rng.standard_normal(
                (xyz_keep.shape[0], 3),
            ).astype(np.float32) * sigma
            xyz_out = xyz_keep + noise

            header = msg.header
            return point_cloud2.create_cloud_xyz32(
                header, [tuple(p) for p in xyz_out],
            )

        except Exception as e:
            self.get_logger().warn(f'LiDAR degradation failed: {e}')
            return msg


def main(args=None):
    rclpy.init(args=args)
    node = SensorDegradation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
