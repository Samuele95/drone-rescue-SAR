#!/usr/bin/env python3
"""Launch the multipage PyQt5 mission dashboard.

The dashboard's "Mission Scene" tab renders the disk, no-fly zones, drone
trails (per-drone color), drone cursors with heading from quaternion, and
victim markers natively in QGraphicsView, with no embedded RViz and no
XEmbed hack. Operators get one window, all tabs.

The no_fly_zones YAML is forwarded so the scene can draw the polygons /
circles in red overlay; defaults to the project's standard config.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    no_fly_arg = DeclareLaunchArgument(
        'no_fly_zones_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('drone_rescue_bringup'),
            'config', 'no_fly_zones.yaml',
        ]),
        description='YAML defining no-fly zones to overlay on the scene',
    )

    dashboard = Node(
        package='drone_rescue_dashboard',
        executable='dashboard',
        name='drone_rescue_dashboard',
        parameters=[{
            'use_sim_time': True,
            'no_fly_zones_yaml': LaunchConfiguration('no_fly_zones_yaml'),
        }],
        output='screen',
    )
    return LaunchDescription([no_fly_arg, dashboard])
