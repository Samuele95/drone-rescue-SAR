#!/usr/bin/env python3
"""
Visualization Launch File

Launches RViz and all visualization nodes for monitoring the drone rescue mission.
Can be run standalone or included from other launch files.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_drone_names(num_drones: int) -> list:
    """Generate list of drone names based on number of drones."""
    return [f'drone{i+1}' for i in range(num_drones)]


def generate_visualization_nodes(context):
    """Generate visualization nodes with dynamic drone names."""
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    drone_names = get_drone_names(num_drones)

    nodes = []

    # Pheromone visualizer
    pheromone_viz = Node(
        package='drone_rescue_viz',
        executable='pheromone_visualizer',
        name='pheromone_visualizer',
        parameters=[{
            'use_sim_time': True,
            'update_rate': 2.0,
            'marker_height': 0.1,
            'marker_alpha': 0.6,
            'min_intensity_threshold': 0.05,
        }],
        output='screen',
    )
    nodes.append(pheromone_viz)

    # Drone trails visualizer
    drone_trails = Node(
        package='drone_rescue_viz',
        executable='drone_trails',
        name='drone_trails',
        parameters=[{
            'use_sim_time': True,
            'drone_names': drone_names,
            'max_trail_points': 500,
            'update_rate': 5.0,
            'trail_width': 0.15,
            'min_distance': 0.3,
        }],
        output='screen',
    )
    nodes.append(drone_trails)

    # Coverage metrics visualizer
    coverage_viz = Node(
        package='drone_rescue_viz',
        executable='coverage_visualizer',
        name='coverage_visualizer',
        parameters=[{
            'use_sim_time': True,
            'text_height': 25.0,
            'text_scale': 1.5,
            'drone_names': drone_names,
        }],
        output='screen',
    )
    nodes.append(coverage_viz)

    # Victim visualizer
    victim_viz = Node(
        package='drone_rescue_viz',
        executable='victim_visualizer',
        name='victim_visualizer',
        parameters=[{
            'use_sim_time': True,
            'marker_lifetime': 0.0,
            'sphere_radius': 0.8,
        }],
        output='screen',
    )
    nodes.append(victim_viz)

    # Telemetry overlay
    telemetry_overlay = Node(
        package='drone_rescue_viz',
        executable='telemetry_overlay',
        name='telemetry_overlay',
        parameters=[{
            'use_sim_time': True,
            'drone_names': drone_names,
        }],
        output='screen',
    )
    nodes.append(telemetry_overlay)

    return nodes


def generate_launch_description():
    # Package directories
    pkg_drone_rescue_bringup = get_package_share_directory('drone_rescue_bringup')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='4',
        description='Number of drones to visualize'
    )

    # RViz config path
    rviz_config = os.path.join(
        pkg_drone_rescue_bringup, 'rviz', 'rescue_mission.rviz'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        num_drones_arg,
        # RViz node
        rviz_node,
        # Visualization nodes with dynamic drone_names
        OpaqueFunction(function=generate_visualization_nodes),
    ])
