#!/usr/bin/env python3
"""
Launch file for earthquake zone simulation.

Launches:
- Gazebo with earthquake zone world (damaged buildings, debris, terrain)
- Drone with sensors
- Environment monitor (weather state machine)
- Zone manager (no-fly zones)
- All ROS-Gazebo bridges
"""

import os
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_drone_model(drone_name: str, template_path: str, output_dir: str) -> str:
    """Generate a drone model SDF with the correct namespace.

    Args:
        drone_name: Name of the drone (e.g., 'drone1')
        template_path: Path to the model.sdf template
        output_dir: Directory to write the generated model

    Returns:
        Path to the generated model file
    """
    # Read template
    with open(template_path, 'r') as f:
        template_content = f.read()

    # Replace placeholder with drone name
    model_content = template_content.replace('${DRONE_NAME}', drone_name)

    # Write to output file
    output_path = os.path.join(output_dir, f'{drone_name}_model.sdf')
    with open(output_path, 'w') as f:
        f.write(model_content)

    return output_path


def generate_launch_description():
    # Package directories
    pkg_drone_rescue_gazebo = get_package_share_directory('drone_rescue_gazebo')
    pkg_drone_rescue_bringup = get_package_share_directory('drone_rescue_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_drone_rescue_gazebo, 'worlds', 'earthquake_zone.sdf'),
        description='Path to the world SDF file'
    )

    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='drone1',
        description='Name/namespace of the drone'
    )

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='X position to spawn the drone'
    )

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position to spawn the drone'
    )

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.5',
        description='Z position to spawn the drone'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    start_controller_arg = DeclareLaunchArgument(
        'start_controller',
        default_value='true',
        description='Start drone controller node'
    )

    start_battery_arg = DeclareLaunchArgument(
        'start_battery',
        default_value='true',
        description='Start battery monitor node'
    )

    start_environment_arg = DeclareLaunchArgument(
        'start_environment',
        default_value='true',
        description='Start environment monitor node'
    )

    start_zones_arg = DeclareLaunchArgument(
        'start_zones',
        default_value='true',
        description='Start zone manager node'
    )

    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='10.0',
        description='Default takeoff altitude in meters'
    )

    enable_weather_changes_arg = DeclareLaunchArgument(
        'enable_weather_changes',
        default_value='true',
        description='Enable dynamic weather changes'
    )

    # Get launch configurations
    drone_name = LaunchConfiguration('drone_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Gazebo with earthquake zone
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r'],
        }.items(),
    )

    # Generate drone model with correct namespace
    # Use 'drone1' as the fixed drone name for this launch file.
    # The template SDF has ${DRONE_NAME} placeholder for robotNamespace
    template_path = os.path.join(pkg_drone_rescue_gazebo, 'models', 'quadrotor', 'model.sdf')
    temp_model_dir = tempfile.mkdtemp(prefix='drone_model_')
    model_path = generate_drone_model('drone1', template_path, temp_model_dir)

    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', drone_name,
            '-file', model_path,
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
        ],
        output='screen',
    )

    # ROS-Gazebo bridges
    # Model uses its spawn name for namespace, so drone1 uses /model/drone1/cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        arguments=[
            '/model/drone1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/model/drone1/cmd_vel', '/drone1/cmd_vel'),
        ],
    )

    bridge_enable = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_enable',
        arguments=[
            '/model/drone1/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/model/drone1/enable', '/drone1/enable'),
        ],
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=[
            '/model/drone1/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/model/drone1/odometry_with_covariance', '/drone1/odom'),
        ],
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=[
            '/world/earthquake_zone/model/drone1/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/world/earthquake_zone/model/drone1/link/base_link/sensor/imu_sensor/imu', '/drone1/imu'),
        ],
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # Drone controller node
    drone_controller = Node(
        package='drone_rescue_coordination',
        executable='drone_controller',
        name='drone_controller',
        parameters=[{
            'drone_name': 'drone1',
            'use_sim_time': True,
            'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            'control_rate': 30.0,
            'position_tolerance': 0.5,
            'max_horizontal_speed': 3.0,
            'max_vertical_speed': 2.0,
            'pid_xy_p': 0.8,
            'pid_xy_i': 0.0,
            'pid_xy_d': 0.3,
            'pid_z_p': 1.0,
            'pid_z_i': 0.0,
            'pid_z_d': 0.4,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_controller')),
    )

    # Battery monitor node
    battery_monitor = Node(
        package='drone_rescue_coordination',
        executable='battery_monitor',
        name='battery_monitor',
        parameters=[{
            'drone_name': 'drone1',
            'use_sim_time': True,
            'initial_level': 1.0,
            'base_drain_rate': 0.0005,
            'movement_drain_factor': 0.0002,
            'low_battery_threshold': 0.2,
            'critical_battery_threshold': 0.1,
        }],
        output='screen',
        remappings=[
            ('/drone/odom', '/drone1/odom'),
        ],
        condition=IfCondition(LaunchConfiguration('start_battery')),
    )

    # Environment monitor node (weather system)
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
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_environment')),
    )

    # Zone manager node (no-fly zones)
    zone_manager = Node(
        package='drone_rescue_coordination',
        executable='zone_manager',
        name='zone_manager',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(pkg_drone_rescue_bringup, 'config', 'no_fly_zones.yaml'),
            'drone_names': ['drone1'],
            'update_rate': 10.0,
            'visualization_enabled': True,
            'warning_distance': 5.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_zones')),
    )

    # Delay node starts to ensure simulation is ready
    delayed_controller = TimerAction(
        period=5.0,
        actions=[drone_controller],
    )

    delayed_battery = TimerAction(
        period=5.0,
        actions=[battery_monitor],
    )

    delayed_environment = TimerAction(
        period=3.0,
        actions=[environment_monitor],
    )

    delayed_zones = TimerAction(
        period=3.0,
        actions=[zone_manager],
    )

    return LaunchDescription([
        # Arguments
        world_arg,
        drone_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        use_sim_time_arg,
        start_controller_arg,
        start_battery_arg,
        start_environment_arg,
        start_zones_arg,
        takeoff_altitude_arg,
        enable_weather_changes_arg,
        # Launch Gazebo
        gz_sim,
        # Spawn drone
        spawn_drone,
        # ROS-Gazebo bridges
        bridge_cmd_vel,
        bridge_enable,
        bridge_odom,
        bridge_imu,
        bridge_clock,
        # Control and monitoring nodes (delayed start)
        delayed_controller,
        delayed_battery,
        delayed_environment,
        delayed_zones,
    ])
