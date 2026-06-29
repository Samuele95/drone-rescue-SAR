#!/usr/bin/env python3
"""
Launch file for drone rescue simulation.

Launches Gazebo with the test world, spawns a drone, and starts control nodes.
"""

import os
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


def generate_launch_description():
    # Package directories
    pkg_drone_rescue_gazebo = get_package_share_directory('drone_rescue_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_drone_rescue_gazebo, 'worlds', 'test_world.sdf'),
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

    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='10.0',
        description='Default takeoff altitude in meters'
    )

    # Get launch configurations
    drone_name = LaunchConfiguration('drone_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r'],
        }.items(),
    )

    # Spawn the drone model
    model_path = os.path.join(pkg_drone_rescue_gazebo, 'models', 'quadrotor', 'model.sdf')

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

    # ROS-Gazebo bridge
    # Bridge Gazebo topics directly to drone1 namespace using separate bridges
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        arguments=[
            '/drone/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bridge_enable = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_enable',
        arguments=[
            # Use ] for ROS->GZ only (prevents feedback loop from Gazebo echoing state)
            '/drone/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
            '/world/test_world/model/drone1/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/world/test_world/model/drone1/link/base_link/sensor/imu_sensor/imu', '/drone1/imu'),
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

    # Drone controller node - uses /drone/ topics directly to match Gazebo
    drone_controller = Node(
        package='drone_rescue_coordination',
        executable='drone_controller',
        name='drone_controller',
        parameters=[{
            'drone_name': 'drone',  # Use 'drone' to match Gazebo namespace
            'use_sim_time': True,
            'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            'control_rate': 30.0,  # Control loop rate
            'position_tolerance': 0.5,
            'max_horizontal_speed': 3.0,
            'max_vertical_speed': 2.0,
            'pid_xy_p': 0.8,
            'pid_xy_i': 0.0,
            'pid_xy_d': 0.3,
            'pid_z_p': 1.0,
            'pid_z_i': 0.0,  # No integral to prevent windup
            'pid_z_d': 0.4,
        }],
        output='screen',
        # Remap odom input to where the bridge publishes it
        remappings=[
            ('/drone/odom', '/drone1/odom'),
        ],
        condition=IfCondition(LaunchConfiguration('start_controller')),
    )

    # Battery monitor node
    battery_monitor = Node(
        package='drone_rescue_coordination',
        executable='battery_monitor',
        name='battery_monitor',
        parameters=[{
            'drone_name': 'drone',  # Match controller
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

    # Delay controller start to ensure bridge is ready
    delayed_controller = TimerAction(
        period=5.0,
        actions=[drone_controller],
    )

    delayed_battery = TimerAction(
        period=5.0,
        actions=[battery_monitor],
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
        takeoff_altitude_arg,
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
        # Control nodes (delayed start)
        delayed_controller,
        delayed_battery,
    ])
