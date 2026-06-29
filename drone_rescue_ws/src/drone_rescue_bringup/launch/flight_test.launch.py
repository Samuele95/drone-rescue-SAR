#!/usr/bin/env python3
"""
Launch file for running the flight test demonstration.

This launch file starts the flight test node which will:
1. Enable motors
2. Takeoff to altitude
3. Navigate through waypoints
4. Return and land

Run this AFTER the simulation is already running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='drone1',
        description='Name of the drone to test'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='10.0',
        description='Takeoff altitude in meters'
    )

    hover_time_arg = DeclareLaunchArgument(
        'hover_time',
        default_value='3.0',
        description='Time to hover at each waypoint (seconds)'
    )

    # Flight test node
    flight_test = Node(
        package='drone_rescue_coordination',
        executable='flight_test',
        name='flight_test',
        parameters=[{
            'drone_name': LaunchConfiguration('drone_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            'hover_time': LaunchConfiguration('hover_time'),
            'waypoint_tolerance': 1.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        drone_name_arg,
        use_sim_time_arg,
        takeoff_altitude_arg,
        hover_time_arg,
        flight_test,
    ])
