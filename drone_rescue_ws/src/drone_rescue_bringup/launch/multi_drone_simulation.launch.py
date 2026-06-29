#!/usr/bin/env python3
"""
Multi-Drone Simulation Launch File

Launches the complete multi-drone survey simulation:
- Gazebo with earthquake zone world
- N drones with controllers, battery monitors, and surveyors
- Pheromone server for stigmergic coordination
- Coverage tracker for mission metrics
- Environment and zone management
"""

import os
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

# Single source of truth for the selectable-algorithm names: the launch-arg
# `choices` below are derived from the registries rather than hardcoded, so a
# newly-registered coverage pattern or allocation strategy needs no edit here.
from drone_rescue_coordination.lib.sar_patterns import CoveragePatternFactory
from drone_rescue_coordination.lib.allocation import AllocationStrategyFactory


# Directory to store generated model files
_temp_model_dir = tempfile.mkdtemp(prefix='drone_models_')

# Color palette for drone visual distinction
DRONE_COLORS = {
    'drone1': {'r': '0.8', 'g': '0.2', 'b': '0.2', 'name': 'red'},
    'drone2': {'r': '0.2', 'g': '0.6', 'b': '0.8', 'name': 'blue'},
    'drone3': {'r': '0.2', 'g': '0.8', 'b': '0.3', 'name': 'green'},
    'drone4': {'r': '0.9', 'g': '0.7', 'b': '0.1', 'name': 'yellow'},
}


def generate_drone_model(drone_name: str, template_path: str) -> str:
    """Generate a drone model SDF with the correct namespace.

    Args:
        drone_name: Name of the drone (e.g., 'drone1')
        template_path: Path to the model.sdf template

    Returns:
        Path to the generated model file
    """
    # Read template
    with open(template_path, 'r') as f:
        template_content = f.read()

    # Replace placeholder with drone name
    model_content = template_content.replace('${DRONE_NAME}', drone_name)

    # Apply drone color
    color = DRONE_COLORS.get(drone_name, {'r': '0.5', 'g': '0.5', 'b': '0.5', 'name': 'gray'})
    model_content = model_content.replace('${DRONE_COLOR_R}', color['r'])
    model_content = model_content.replace('${DRONE_COLOR_G}', color['g'])
    model_content = model_content.replace('${DRONE_COLOR_B}', color['b'])

    print(f"  Spawning {drone_name} with color: {color['name']}")

    # Write to temp file
    output_path = os.path.join(_temp_model_dir, f'{drone_name}_model.sdf')
    with open(output_path, 'w') as f:
        f.write(model_content)

    return output_path


def spawn_drone(context, drone_id, spawn_x, spawn_y, spawn_yaw):
    """Generate nodes for a single drone."""
    pkg_drone_rescue_gazebo = get_package_share_directory('drone_rescue_gazebo')

    drone_name = f'drone{drone_id}'
    template_path = os.path.join(pkg_drone_rescue_gazebo, 'models', 'quadrotor', 'model.sdf')

    # Generate model file with correct namespace
    model_path = generate_drone_model(drone_name, template_path)

    nodes = []

    # Spawn drone model
    spawn_drone_node = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{drone_name}',
        arguments=[
            '-name', drone_name,
            '-file', model_path,
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', '0.5',
            '-Y', str(spawn_yaw),
        ],
        output='screen',
    )
    nodes.append(spawn_drone_node)

    # ROS-Gazebo bridges for this drone
    # MulticopterVelocityControl publishes odometry_with_covariance
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_odom_{drone_name}',
        arguments=[
            f'/model/{drone_name}/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            (f'/model/{drone_name}/odometry_with_covariance', f'/{drone_name}/odom'),
        ],
    )
    nodes.append(bridge_odom)

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_imu_{drone_name}',
        arguments=[
            f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            (f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/imu_sensor/imu', f'/{drone_name}/imu'),
        ],
    )
    nodes.append(bridge_imu)

    # Command velocity bridge for this drone
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_cmd_vel_{drone_name}',
        arguments=[
            # ROS->GZ only (']'). A bidirectional ('@') bridge on this actuator
            # topic lets Gazebo mirror state back onto the controller's own
            # command topic; cmd_vel is command-only.
            f'/model/{drone_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            (f'/model/{drone_name}/cmd_vel', f'/{drone_name}/cmd_vel'),
        ],
    )
    nodes.append(bridge_cmd_vel)

    # Enable bridge for this drone
    bridge_enable = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_enable_{drone_name}',
        arguments=[
            # ROS->GZ only (']'). A bidirectional ('@') bridge here mirrors the
            # Gazebo-side enable state back onto /{drone}/enable, which the
            # controller also subscribes to, a feedback loop that produces the
            # rapid Motors ENABLED/DISABLED flapping. enable is command-only.
            f'/model/{drone_name}/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            (f'/model/{drone_name}/enable', f'/{drone_name}/enable'),
        ],
    )
    nodes.append(bridge_enable)

    # Camera bridge: exposes the downward RGB image to the global victim_detector.
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_camera_{drone_name}',
        arguments=[
            f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            (
                f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/camera/image',
                f'/{drone_name}/camera',
            ),
        ],
    )
    nodes.append(bridge_camera)

    # Keep a persistent gz-side subscriber on each camera topic for the
    # launch lifetime. Mirrors the /clock prime: the ros_gz_bridge has a
    # per-topic race where the ROS-side stays silent until something on the
    # gz side first subscribes AND stays subscribed. Without these primes,
    # 4 of 8 camera streams typically end up dark in any given launch
    # (which side wins is non-deterministic across runs). The echoed stream
    # is discarded to /dev/null (we only need the subscription alive, not its
    # payload): ``gz topic -e`` dumps full image protobufs at ~9 MB/s, which
    # previously flooded launch.log (fatal under a containerised, size-bounded
    # log dir). ``exec`` keeps the process tree flat so launch can terminate
    # it on shutdown.
    prime_camera = TimerAction(
        period=6.0,   # wait for gz to be ready + the bridge to register
        actions=[ExecuteProcess(
            cmd=['sh', '-c',
                 'exec gz topic -e -t '
                 f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/camera/image'
                 ' >/dev/null 2>&1'],
            output='log',
            shell=False,
        )],
    )
    nodes.append(prime_camera)

    # Follow-camera bridge: third-person view of the drone for the dashboard.
    bridge_follow_cam = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_follow_cam_{drone_name}',
        arguments=[
            f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/follow_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            (
                f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/follow_camera/image',
                f'/{drone_name}/follow_cam',
            ),
        ],
    )
    nodes.append(bridge_follow_cam)

    # Same prime for the follow-camera bridge. See the prime_camera comment above.
    prime_follow_cam = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['sh', '-c',
                 'exec gz topic -e -t '
                 f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/follow_camera/image'
                 ' >/dev/null 2>&1'],
            output='log',
            shell=False,
        )],
    )
    nodes.append(prime_follow_cam)

    # LiDAR bridge: 360° GPU lidar already exists in model.sdf but was never
    # bridged. drone_health_monitor needs the min-range for collision-imminent
    # anomaly detection; detection_filter uses the same scan to corroborate
    # candidate sightings against actual ground returns.
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_lidar_{drone_name}',
        arguments=[
            f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            (
                f'/world/earthquake_zone/model/{drone_name}/link/base_link/sensor/lidar/scan',
                f'/{drone_name}/scan',
            ),
        ],
    )
    nodes.append(bridge_lidar)

    return nodes


def spawn_drone_controllers(context, drone_id, spawn_x, spawn_y, all_drone_names):
    """Generate controller and support nodes for a single drone."""
    drone_name = f'drone{drone_id}'

    nodes = []

    # Drone controller - Lifecycle node
    controller = LifecycleNode(
        package='drone_rescue_coordination',
        executable='drone_controller',
        name=f'{drone_name}_controller',
        namespace='',
        parameters=[{
            'drone_name': drone_name,
            'use_sim_time': True,
            'takeoff_altitude': 25.0,
            'control_rate': 30.0,
            'position_tolerance': 0.5,
            # The executor publishes a setpoint only step_m = survey_speed *
            # (1/control_rate_used_by_executor) * 4 ~= 2 m ahead of the drone's
            # current pose, so the steady-state cruise of the P-controller is
            # pid_xy_p * step_m. At the old 0.8 that capped ground speed at
            # ~1.6 m/s, below survey_speed (2.5), so the ~890 m spiral could
            # not be flown inside mission_timeout (600 s) and drones never
            # reached the outer victims. Restore the controller's intended
            # gains/clamp: pid_xy_p 1.5 (+ a little I to kill the tracking lag)
            # and the 5 m/s clamp, giving ~3 m/s effective.
            'max_horizontal_speed': 5.0,
            'max_vertical_speed': 2.0,
            'pid_xy_p': 1.5,
            'pid_xy_i': 0.2,
            'pid_xy_d': 0.3,
            'pid_z_p': 1.0,
            'pid_z_i': 0.0,
            'pid_z_d': 0.4,
            # Software wind model. disturbance 1.0 = the wind actually
            # pushes the drone; compensation 0.5 = partial station-keeping, so
            # wind still visibly moves the drone but drifts it less than an
            # uncompensated drone (set compensation to 0.0 to demo the full
            # drift, or to 1.0 to hold station). Only acts when the scenario's
            # weather is windy (wind vector non-zero).
            'wind_disturbance_gain': 1.0,
            'wind_compensation_gain': 0.5,
        }],
        output='screen',
    )
    nodes.append(controller)

    # Battery monitor
    battery = Node(
        package='drone_rescue_coordination',
        executable='battery_monitor',
        name=f'{drone_name}_battery',
        parameters=[{
            'drone_name': drone_name,
            'use_sim_time': True,
            'initial_level': 1.0,
            'base_drain_rate': 0.0003,
            'movement_drain_factor': 0.0001,
            'low_battery_threshold': 0.2,
            'critical_battery_threshold': 0.1,
        }],
        output='screen',
    )
    nodes.append(battery)

    # Drone executor (Behavior-Tree task runner; replaces surveyor), one per drone.
    drone_executor = LifecycleNode(
        package='drone_rescue_coordination',
        executable='drone_executor',
        name=f'{drone_name}_executor',
        namespace='',
        parameters=[{
            'drone_name': drone_name,
            'use_sim_time': True,
            'tick_rate_hz': 5.0,
            'survey_altitude': 25.0,
            'survey_speed': 2.5,
            'position_tolerance_m': 1.5,
            'min_takeoff_altitude_m': 18.0,
            'escape_altitude_m': 35.0,  # backup climb if even higher obstacle
            'mission_center_x': 0.0,
            'mission_center_y': 0.0,
            'mission_radius': 85.0,
            'peer_drone_names': all_drone_names,
        }],
        output='screen',
    )
    nodes.append(drone_executor)

    # Per-drone health monitor. Watches IMU/odom/scan/battery
    # and emits /<drone>/health at 5 Hz; on hard fail emits a one-shot
    # DRONE_DAMAGE_REPORT mission event.
    drone_health = Node(
        package='drone_rescue_coordination',
        executable='drone_health_monitor',
        name=f'{drone_name}_health',
        namespace='',
        parameters=[{
            'drone_name': drone_name,
            'use_sim_time': True,
            'publish_rate_hz': 5.0,
            'imu_spike_g': 30.0,
            'odom_stale_s': 2.0,
            'vel_freeze_m_s': 0.2,
            'vel_freeze_window_s': 4.0,
            'lidar_imminent_m': 1.0,
            'lidar_imminent_window_s': 1.0,
            # Require 3 simultaneous anomalies to declare DOWN. With only 2,
            # a brief stuck-in-zone moment + lidar grazing trips the latch
            # too aggressively. Three of {imu_spike, vel_freeze, pos_freeze,
            # lidar_imminent, battery_critical} = solid evidence the drone
            # is genuinely lost.
            'unrecoverable_anomaly_count': 3,
            'grounded_altitude_m': 5.0,
        }],
        output='screen',
    )
    nodes.append(drone_health)

    return nodes


def generate_drone_nodes(context):
    """Generate all drone-related nodes based on num_drones parameter."""
    num_drones = int(LaunchConfiguration('num_drones').perform(context))

    # Spawn positions for drones (spread around landing pad).
    # All drones spawn with yaw=0 so each drone's odom frame is aligned with
    # world ENU. The boustrophedon waypoints from mission_manager are in the
    # mission_center frame (world); without this alignment, drone N would
    # interpret a "fly to (0, 85)" command in its own rotated frame and end up
    # off-target, drone2 in particular used to drift west when commanded to
    # fly north because its spawn yaw of +π/2 confused the body-frame rotator.
    spawn_configs = [
        (0.0, 0.0, 0.0),       # drone1
        (2.0, 0.0, 0.0),       # drone2
        (0.0, 2.0, 0.0),       # drone3
        (2.0, 2.0, 0.0),       # drone4
        (-2.0, 0.0, 0.0),      # drone5
        (-2.0, 2.0, 0.0),      # drone6
        (4.0, 0.0, 0.0),       # drone7
        (4.0, 2.0, 0.0),       # drone8
    ]

    all_nodes = []

    for i in range(min(num_drones, len(spawn_configs))):
        drone_id = i + 1
        spawn_x, spawn_y, spawn_yaw = spawn_configs[i]

        # Add spawn and bridge nodes
        all_nodes.extend(spawn_drone(context, drone_id, spawn_x, spawn_y, spawn_yaw))

    return all_nodes


def generate_controller_nodes(context):
    """Generate controller nodes after a delay."""
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    all_drone_names = get_drone_names(num_drones)

    spawn_configs = [
        (0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0),
        (-2.0, 0.0), (-2.0, 2.0), (4.0, 0.0), (4.0, 2.0),
    ]

    all_nodes = []

    for i in range(min(num_drones, len(spawn_configs))):
        drone_id = i + 1
        spawn_x, spawn_y = spawn_configs[i]
        all_nodes.extend(spawn_drone_controllers(context, drone_id, spawn_x, spawn_y, all_drone_names))

    return all_nodes


def get_drone_names(num_drones: int) -> list:
    """Generate list of drone names based on number of drones."""
    return [f'drone{i+1}' for i in range(num_drones)]


def generate_coordination_nodes(context):
    """Generate coordination nodes with dynamic drone names."""
    pkg_drone_rescue_bringup = get_package_share_directory('drone_rescue_bringup')
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    drone_names = get_drone_names(num_drones)

    # Terrain gradient: resolved to real floats so mission_manager and
    # victim_detector build identical planar ElevationModels (the detector must
    # measure AGL). Both 0.0 (default) = flat terrain. The matching
    # Gazebo world (sloped_terrain.yaml -> earthquake_zone_sloped.sdf) tilts the
    # ground plane to the same gradient.
    terrain_slope_x = float(LaunchConfiguration('terrain_slope_x').perform(context))
    terrain_slope_y = float(LaunchConfiguration('terrain_slope_y').perform(context))

    nodes = []

    # Coverage tracker
    coverage_tracker = Node(
        package='drone_rescue_coordination',
        executable='coverage_tracker',
        name='coverage_tracker',
        parameters=[{
            'use_sim_time': True,
            'update_rate': 1.0,
            'drone_names': drone_names,
            'coverage_threshold': 0.1,
            'grid_width': 400,
            'grid_height': 400,
            'cell_resolution': 1.0,
            'origin_x': -200.0,
            'origin_y': -200.0,
            'mission_center_x': 0.0,
            'mission_center_y': 0.0,
            'mission_radius': 85.0,
        }],
        output='screen',
    )
    nodes.append(coverage_tracker)

    # Victim detector: now publishes per-drone /detections_raw only.
    victim_detector = Node(
        package='drone_rescue_coordination',
        executable='victim_detector',
        name='victim_detector',
        parameters=[{
            'use_sim_time': True,
            'drone_names': drone_names,
            'detection_rate': 2.0,
            'min_detection_height': 3.0,
            'max_detection_height': 50.0,
            # Match the widened 90° camera in model.sdf: projection from
            # pixel to ground requires the right FOV or the computed victim
            # position drifts.
            'camera_fov_horizontal': 90.0,
            # Was 40. At 25 m survey altitude with
            # 90° FOV and 480×360 res, a 0.5×0.8 m victim torso renders
            # as ~5×8 px = ~40 px solid area. After MORPH_CLOSE (the
            # MORPH_OPEN was dropped, see victim_detector.py) the
            # connected component lands in the ~30-60 px range; 40 was
            # the boundary that rejected most real victims. 5 lets
            # genuine victims through; false positives are gated
            # downstream by detection_filter's multi-view confirmation.
            'min_contour_area': 5,
            'max_contour_area': 2000,
            'debug_detection': False,
            # Decay detection confidence by horizontal distance to the drone.
            # Inside ``range_decay_start_m``: full confidence. Beyond
            # ``max_detection_range_m``: drop the detection entirely.
            # Default (5 / 12 m) forces drones to overfly within ~12 m
            # to even register a candidate, and within ~5 m to score
            # high enough for the saga's INVESTIGATE path. See the
            # ``_apply_range_decay`` docstring in victim_detector.py
            # for the full rationale.
            'max_detection_range_m': 12.0,
            'range_decay_start_m': 5.0,
            # Same terrain gradient as mission_manager so the
            # detector's AGL gate matches the AGL flight altitude.
            'terrain_slope_x': terrain_slope_x,
            'terrain_slope_y': terrain_slope_y,
        }],
        output='screen',
    )
    nodes.append(victim_detector)

    # Detection filter: DBSCAN + Bayesian fusion + multi-view confirmation.
    detection_filter = Node(
        package='drone_rescue_coordination',
        executable='detection_filter',
        name='detection_filter',
        parameters=[{
            'use_sim_time': True,
            'drone_names': drone_names,
            # Was 9.0. The original 9 m self-filter
            # was calibrated for 14 m altitude (half of the 16 m
            # footprint at 60° FOV). With the current 25 m altitude +
            # 90° FOV, the footprint radius is 25 m and 9 m rejects
            # the central 36 % of every frame, exactly the part where
            # a drone hovering over a victim during INVESTIGATE
            # projects the victim to the drone's own XY. 2 m only
            # rejects sightings literally underneath the drone body
            # (where the colored frame might leak into the camera).
            'min_distance_from_drones': 2.0,
            # Higher confidence floor reduces false positives from world
            # textures (red building roofs, signs, vehicles) that match the
            # detector's HSV thresholds.
            'confidence_floor': 0.65,
            'cluster_window_seconds': 45.0,
            'dbscan_eps_m': 6.0,
            'dbscan_min_samples': 2,
            'confirmation_threshold': 0.8,
            # Require at least 5 sightings in a cluster before auto-confirm.
            # Two drones happening to fly past the same red roof produce 2-3
            # sightings, enough for DBSCAN, not enough to confirm. Real
            # victims accumulate sightings as the mission INVESTIGATEs them
            # with hovering drones, so the bar clears within seconds.
            'min_confirm_observations': 5,
            # Require ≥2 drones each with ≥2 sightings, paired with
            # mission_manager auctioning CONFIRM to a *different* drone
            # than the INVESTIGATEr, this means a confirmation requires
            # two physically distinct viewpoints. A single drone hovering
            # over a textured wall can no longer self-confirm.
            'min_multi_witnesses': 2,
            'min_sightings_per_witness': 2,
            'publish_rate_hz': 2.0,
        }],
        output='screen',
    )
    nodes.append(detection_filter)

    # Mission manager: Lifecycle node that orchestrates the SAR mission.
    mission_manager = LifecycleNode(
        package='drone_rescue_coordination',
        executable='mission_manager',
        name='mission_manager',
        namespace='',
        parameters=[{
            'use_sim_time': True,
            'drone_names': drone_names,
            'mission_center_x': 0.0,
            'mission_center_y': 0.0,
            'mission_radius': 70.0,  # reach v4(55,48) but skip the empty rim
            # Cruise altitude ABOVE the world's tallest structure (24m). This
            # means the LiDAR-driven escape climb is now a safety backup, not
            # the primary collision-avoidance strategy.
            'survey_altitude': 25.0,
            # Real footprint at 25m alt × 90° FOV = 50m theoretical.
            # Use 35m conservative: accounts for fish-eye distortion at the
            # edges where pixel/meter ratio drops and detection accuracy
            # falls off. Real SAR drones use the central ~70% of the image
            # for reliable detection; 35m matches that.
            'camera_footprint_m': 35.0,
            # 85% overlap -> track_spacing = 5.25m -> ~14 arcs every 5m
            # across the disk. The visible trail spacing on the Mission
            # Scene tab now matches the operator's intuition of "every
            # square metre is overflown".
            'coverage_overlap': 0.85,
            'coverage_pattern': LaunchConfiguration('coverage_pattern'),
            'allocation_strategy': LaunchConfiguration('allocation_strategy'),
            # Start arcs near the launch pad (5 m) instead of 15 m so coverage
            # begins immediately around the base. The previous 15 m donut hole
            # left an obvious uncovered ring around the centre on the dashboard.
            'inner_radius': 5.0,
            'investigate_hover_seconds': 4.0,
            'confirm_hover_seconds': 6.0,
            'confirm_orbit_radius': 4.0,
            'reject_age_seconds': 60.0,
            'mission_timeout_seconds': 600.0,
            'seed': LaunchConfiguration('seed'),
            # Terrain gradient for the planar ElevationModel; scan
            # waypoints fly at constant AGL (survey_altitude + elevation_at).
            'terrain_slope_x': terrain_slope_x,
            'terrain_slope_y': terrain_slope_y,
        }],
        output='screen',
    )
    nodes.append(mission_manager)

    # Zone manager
    zone_manager = Node(
        package='drone_rescue_coordination',
        executable='zone_manager',
        name='zone_manager',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(pkg_drone_rescue_bringup, 'config', 'no_fly_zones.yaml'),
            'drone_names': drone_names,
            'update_rate': 10.0,
            'visualization_enabled': True,
            'warning_distance': 5.0,
        }],
        output='screen',
    )
    nodes.append(zone_manager)

    # Readiness coordinator - event-driven survey start
    readiness_coordinator = Node(
        package='drone_rescue_coordination',
        executable='readiness_coordinator',
        name='readiness_coordinator',
        parameters=[{
            'use_sim_time': True,
            'drone_names': drone_names,
            'min_odom_count': 10,
            'odom_timeout': 2.0,
            'min_ready_duration': 5.0,
        }],
        output='screen',
    )
    nodes.append(readiness_coordinator)

    return nodes


def generate_lifecycle_manager(context):
    """Generate lifecycle manager node with dynamic drone names."""
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    drone_names = get_drone_names(num_drones)

    return [Node(
        package='drone_rescue_coordination',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'use_sim_time': True,
            'drone_names': drone_names,
            'transition_timeout': 10.0,
            'auto_startup': True,
        }],
        output='screen',
    )]


def generate_diagnostics_aggregator(context):
    """Generate diagnostic aggregator node."""
    pkg_drone_rescue_bringup = get_package_share_directory('drone_rescue_bringup')

    diagnostics_config = os.path.join(
        pkg_drone_rescue_bringup, 'config', 'diagnostics_aggregator.yaml')

    return [Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        parameters=[diagnostics_config],
        output='screen',
    )]


def generate_bag_recording(context):
    """Generate bag recording process if enabled."""
    record_bag = LaunchConfiguration('record_bag').perform(context)

    if record_bag.lower() != 'true':
        return []

    pkg_drone_rescue_bringup = get_package_share_directory('drone_rescue_bringup')

    # Topics to record (inline for reliability, config file as backup)
    topics = [
        '/diagnostics',
        '/diagnostics_agg',
        '/diagnostics_toplevel_state',
        '/drone1/status',
        '/drone2/status',
        '/drone3/status',
        '/drone4/status',
        '/pheromone/coverage',
        '/survey/start',
        '/victims/detected',
        '/environment/weather',
    ]

    # Create timestamped bag name
    from datetime import datetime
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = f'rosbag2_diagnostics_{timestamp}'

    # Build command
    cmd = ['ros2', 'bag', 'record', '-o', bag_path] + topics

    return [ExecuteProcess(
        cmd=cmd,
        name='rosbag_record',
        output='screen',
    )]


def generate_launch_description():
    # Package directories
    pkg_drone_rescue_gazebo = get_package_share_directory('drone_rescue_gazebo')
    pkg_drone_rescue_bringup = get_package_share_directory('drone_rescue_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Force FastDDS to use UDP transport instead of shared memory.
    # Shared memory transport fails with 40+ DDS participants (bridges + nodes).
    fastdds_profile = os.path.join(pkg_drone_rescue_bringup, 'config', 'fastdds_profile.xml')
    set_fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile
    )

    # Launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='4',
        description='Number of drones to spawn (1-8)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_drone_rescue_gazebo, 'worlds', 'earthquake_zone.sdf'),
        description='Path to the world SDF file'
    )

    enable_weather_arg = DeclareLaunchArgument(
        'enable_weather_changes',
        default_value='true',
        description='Enable dynamic weather changes'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Enable rosbag recording of diagnostics topics'
    )

    coverage_pattern_arg = DeclareLaunchArgument(
        'coverage_pattern',
        default_value='spiral_out',
        description=(
            'SAR coverage strategy. Registered names come from '
            'CoveragePatternFactory; default spiral_out = concentric arcs '
            'from the launch pad outward.'
        ),
        choices=CoveragePatternFactory.list_names(),
    )

    allocation_strategy_arg = DeclareLaunchArgument(
        'allocation_strategy',
        default_value='greedy_auction',
        description=(
            'Task-allocation strategy for victim INVESTIGATE/CONFIRM '
            'dispatch. Registered names come from AllocationStrategyFactory; '
            'default greedy_auction = nearest-drone greedy auction.'
        ),
        choices=AllocationStrategyFactory.list_names(),
    )

    seed_arg = DeclareLaunchArgument(
        'seed',
        default_value='0',
        description=(
            'Master RNG seed for reproducible runs (V5). Forwarded as a '
            'ROS param to environment_monitor (weather/wind), '
            'sensor_degradation (camera/LiDAR noise), mission_manager '
            '(auction tie-break + RandomWalkPattern). Each node offsets '
            'the parent seed internally so streams stay independent. '
            'Two runs with the same seed produce numerically identical '
            'JSONL summaries (timestamps and Gazebo physics aside — see '
            'docs/v5-release.md for the determinism caveat).'
        ),
    )

    terrain_slope_x_arg = DeclareLaunchArgument(
        'terrain_slope_x',
        default_value='0.0',
        description=(
            'Terrain elevation gradient along world +x in m/m (P1-2). '
            '0.0 = flat. Non-zero tilts the planar ElevationModel so scan '
            'waypoints fly at constant AGL and the detector measures AGL; '
            'pair it with a Gazebo world whose ground plane is tilted to the '
            'same gradient (see scenarios/sloped_terrain.yaml).'
        ),
    )

    terrain_slope_y_arg = DeclareLaunchArgument(
        'terrain_slope_y',
        default_value='0.0',
        description='Terrain elevation gradient along world +y in m/m (P1-2).',
    )

    record_run_arg = DeclareLaunchArgument(
        'record_run',
        default_value='false',
        description=(
            'Spawn the mission_recorder node, which writes a JSONL '
            'summary on MISSION_COMPLETE / MISSION_TIMEOUT / SIGTERM. '
            'Mission Control sets this to true; bare ros2 launch leaves '
            'it false so a quick smoke test does not pollute runs/.'
        ),
        choices=['true', 'false'],
    )

    scenario_yaml_arg = DeclareLaunchArgument(
        'scenario_yaml',
        default_value='',
        description=(
            'Path to the scenario YAML used for this run. The recorder '
            'reads it for ground_truth_victims (TP/FP scoring) and saves '
            'a snapshot of params into the JSONL.'
        ),
    )

    scenario_name_arg = DeclareLaunchArgument(
        'scenario_name',
        default_value='unknown',
        description='Short scenario name written into the JSONL filename.',
    )

    runs_dir_arg = DeclareLaunchArgument(
        'runs_dir',
        default_value=os.path.expanduser('~/.drone_rescue/runs'),
        description='Directory where the recorder writes per-run JSONLs.',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description=(
            'Run Gazebo server-only (no gz-sim GUI window). The operator '
            'GUI is the dashboard / Mission Control; the Gazebo client '
            'window is redundant for normal use and its orphaned-on-'
            'SIGTERM children leak processes after stop. Set false for '
            'debugging the physics scene directly in the gz-sim window.'
        ),
        choices=['true', 'false'],
    )

    # Launch Gazebo. ``-r`` runs (not paused); ``-s`` adds server-only
    # when headless. The world value may be an absolute SDF path (the
    # default) or a bare filename: a scenario YAML's ``world:`` key
    # typically gives just ``earthquake_zone_sparse.sdf``; we resolve
    # bare names against the gazebo package's ``worlds/`` directory so
    # scenarios needn't hard-code an install-tree path. Absolute,
    # existing paths (the default and every world-less scenario) are
    # passed through unchanged.
    def _launch_gz_sim(context, *args, **kwargs):
        world = LaunchConfiguration('world').perform(context)
        headless = LaunchConfiguration('headless').perform(context)
        if not (os.path.isabs(world) and os.path.isfile(world)):
            candidate = world if world.endswith('.sdf') else world + '.sdf'
            resolved = os.path.join(
                pkg_drone_rescue_gazebo, 'worlds', os.path.basename(candidate))
            if os.path.isfile(resolved):
                world = resolved
        gz_args = (f'{world} -r -s' if headless.lower() == 'true'
                   else f'{world} -r')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        )]

    gz_sim = OpaqueFunction(function=_launch_gz_sim)

    # Clock bridge
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Keep a persistent gz-side subscriber on /clock for the lifetime of
    # the launch. The ros_gz_bridge parameter_bridge has a known race where
    # the ROS-side topic stays silent until something on the gz side
    # subscribes (then the bridge forwards), and empirically the bridge
    # requires the subscriber to stay alive, not just appear once. Without
    # this, lifecycle_manager waits for /clock to tick before driving
    # on_configure transitions; the wait times out after 10 s, every
    # lifecycle node stays unconfigured, mission_manager never receives
    # /survey/start, _begin_scan is never called, and the saga degenerates
    # into a pure-reactive victim chase with no coverage plan (the mission
    # saga carries a separate defence for this). 4 s delay lets gz fully
    # initialise before we subscribe; the subprocess is terminated
    # automatically when the launch tree shuts down.
    prime_clock_bridge = TimerAction(
        period=4.0,   # wait for gz to be ready
        actions=[ExecuteProcess(
            # discard the echoed /clock dump; only the subscription matters
            cmd=['sh', '-c', 'exec gz topic -e -t /clock >/dev/null 2>&1'],
            output='log',
            shell=False,
        )],
    )

    # Pheromone server (single instance) - Lifecycle node
    pheromone_server = LifecycleNode(
        package='drone_rescue_coordination',
        executable='pheromone_server',
        name='pheromone_server',
        namespace='',
        parameters=[{
            'use_sim_time': True,
            'grid_width': 400,
            'grid_height': 400,
            'cell_resolution': 1.0,
            'origin_x': -200.0,
            'origin_y': -200.0,
            'decay_rate': 0.992,
            'update_rate': 2.0,
            'deposit_value': 1.0,
            'coverage_threshold': 0.1,
            'deposit_radius_cells': 4,
            'deposit_sigma_cells': 2.0,
        }],
        output='screen',
    )

    # Environment monitor
    environment_monitor = Node(
        package='drone_rescue_coordination',
        executable='environment_monitor',
        name='environment_monitor',
        parameters=[{
            'use_sim_time': True,
            'update_rate': 1.0,
            'weather_change_interval': 120.0,
            'enable_weather_changes': LaunchConfiguration('enable_weather_changes'),
            'initial_weather': 'clear',
            'clear_wind_speed': 2.0,
            'windy_wind_speed': 8.0,
            'storm_wind_speed': 15.0,
            'seed': LaunchConfiguration('seed'),
        }],
        output='screen',
    )

    # Spawn drones (immediately after Gazebo starts)
    spawn_drones = OpaqueFunction(function=generate_drone_nodes)

    # Delayed start for controllers (wait for simulation to stabilize)
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=generate_controller_nodes)],
    )

    # Lifecycle manager starts after all lifecycle nodes spawned
    delayed_lifecycle_manager = TimerAction(
        period=8.0,  # After controllers (5.0) + buffer for node spawn
        actions=[OpaqueFunction(function=generate_lifecycle_manager)],
    )

    # Diagnostic aggregator (after lifecycle nodes publish diagnostics)
    delayed_diagnostics_aggregator = TimerAction(
        period=10.0,  # After lifecycle manager (8.0) + buffer
        actions=[OpaqueFunction(function=generate_diagnostics_aggregator)],
    )

    # Delayed start for coordination nodes
    delayed_pheromone = TimerAction(
        period=3.0,
        actions=[pheromone_server],
    )

    delayed_environment = TimerAction(
        period=3.0,
        actions=[environment_monitor],
    )

    # Coordination nodes with dynamic drone names (coverage_tracker + zone_manager)
    delayed_coordination = TimerAction(
        period=4.0,
        actions=[OpaqueFunction(function=generate_coordination_nodes)],
    )

    # Visualization launch (delayed to let simulation start)
    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drone_rescue_bringup, 'launch', 'visualization.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'num_drones': LaunchConfiguration('num_drones'),
        }.items(),
    )

    delayed_visualization = TimerAction(
        period=6.0,
        actions=[visualization_launch],
    )

    # Bag recording (delayed to ensure topics exist)
    delayed_bag_recording = TimerAction(
        period=15.0,  # After aggregator (10.0) + buffer for first diagnostics
        actions=[OpaqueFunction(function=generate_bag_recording)],
    )

    # ReadinessCoordinator handles survey start automatically when all drones are ready
    # (Removed hardcoded 120s delay - now event-driven)

    # Mission recorder: only spawned when record_run:=true. Runs alongside
    # the rest of the stack and writes a JSONL summary on MISSION_COMPLETE
    # / MISSION_TIMEOUT / SIGTERM. Mission Control turns this on per run.
    mission_recorder = Node(
        package='drone_rescue_mission_control',
        executable='mission_recorder',
        name='mission_recorder',
        condition=IfCondition(LaunchConfiguration('record_run')),
        parameters=[{
            'use_sim_time': True,
            'runs_dir': LaunchConfiguration('runs_dir'),
            'scenario_yaml': LaunchConfiguration('scenario_yaml'),
            'scenario_name': LaunchConfiguration('scenario_name'),
            'coverage_pattern': LaunchConfiguration('coverage_pattern'),
            'allocation_strategy': LaunchConfiguration('allocation_strategy'),
        }],
        output='screen',
    )
    delayed_recorder = TimerAction(
        period=10.0,   # well after lifecycle activation
        actions=[mission_recorder],
    )

    return LaunchDescription([
        # Environment
        set_fastdds_profile,
        # Arguments
        num_drones_arg,
        world_arg,
        enable_weather_arg,
        use_rviz_arg,
        record_bag_arg,
        coverage_pattern_arg,
        allocation_strategy_arg,
        seed_arg,
        terrain_slope_x_arg,
        terrain_slope_y_arg,
        record_run_arg,
        scenario_yaml_arg,
        scenario_name_arg,
        runs_dir_arg,
        headless_arg,
        # Recorder (only fires if record_run=true)
        delayed_recorder,
        # Gazebo
        gz_sim,
        # Bridges
        bridge_clock,
        # Prime the /clock bridge so lifecycle_manager's startup doesn't
        # hang waiting for /clock to tick (see the comment on the prime
        # action itself).
        prime_clock_bridge,
        # Spawn drones
        spawn_drones,
        # Coordination nodes (delayed)
        delayed_pheromone,
        delayed_environment,
        delayed_coordination,  # coverage_tracker + zone_manager with dynamic drone_names
        # Controllers (delayed more)
        delayed_controllers,
        # Lifecycle manager (after lifecycle nodes spawned)
        delayed_lifecycle_manager,
        # Diagnostics aggregator
        delayed_diagnostics_aggregator,
        # Visualization (delayed more)
        delayed_visualization,
        # Bag recording (delayed to ensure topics exist)
        delayed_bag_recording,
        # ReadinessCoordinator added via generate_coordination_nodes()
    ])
