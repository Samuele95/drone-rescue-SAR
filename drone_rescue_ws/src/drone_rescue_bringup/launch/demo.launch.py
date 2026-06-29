#!/usr/bin/env python3
"""
Demo Launch File - Complete Phase 2 Storytelling Demo

Integrates:
- Multi-drone simulation (Gazebo + drones + coordination + telemetry)
- Camera director with scripted shot sequence
- Video recording capabilities
- RViz visualization with telemetry overlays

Single command to launch the complete demo:
    ros2 launch drone_rescue_bringup demo.launch.py

The camera director starts with a delay (~12s) to allow Gazebo GUI and drones
to initialize before executing the automated shot sequence.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for complete demo."""

    pkg_drone_rescue_bringup = get_package_share_directory('drone_rescue_bringup')

    # Launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='4',
        description='Number of drones to spawn (1-8)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization with telemetry overlays'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable automated camera director shot sequence'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Enable rosbag recording of diagnostics topics'
    )

    demo_timeout_arg = DeclareLaunchArgument(
        'demo_timeout',
        default_value='480.0',
        description='Sim-time timeout in seconds for forced demo completion (default 480s = 8 min)'
    )

    coverage_completion_arg = DeclareLaunchArgument(
        'coverage_completion',
        default_value='60.0',
        description='Coverage percentage threshold to trigger completion shot (default 60%)'
    )

    coverage_survey_arg = DeclareLaunchArgument(
        'coverage_survey',
        default_value='20.0',
        description='Coverage percentage threshold to trigger survey shot (default 20%)'
    )

    coverage_pattern_arg = DeclareLaunchArgument(
        'coverage_pattern',
        default_value='spiral_out',
        description=(
            'SAR coverage strategy — see CoveragePatternFactory for the '
            'registered names (validated by multi_drone_simulation.launch.py).'
        ),
    )

    allocation_strategy_arg = DeclareLaunchArgument(
        'allocation_strategy',
        default_value='greedy_auction',
        description=(
            'Task-allocation strategy — see AllocationStrategyFactory for '
            'the registered names (validated by '
            'multi_drone_simulation.launch.py).'
        ),
    )

    seed_arg = DeclareLaunchArgument(
        'seed', default_value='0',
        description=(
            'Master RNG seed (V5). Forwarded to environment_monitor, '
            'sensor_degradation, mission_manager. See multi_drone_simulation '
            'launch for the per-node offset scheme.'
        ),
    )

    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description=(
            'Spawn the unified rqt mission dashboard (per-drone state '
            'table + mission log + 8 camera tiles + embedded RViz). '
            'When true, the standalone RViz is suppressed (rqt embeds it).'
        ),
        choices=['true', 'false'],
    )

    record_run_arg = DeclareLaunchArgument(
        'record_run', default_value='false',
        description='Spawn mission_recorder; writes a JSONL summary on completion.',
        choices=['true', 'false'],
    )
    scenario_yaml_arg = DeclareLaunchArgument(
        'scenario_yaml', default_value='',
        description='Scenario YAML for ground truth + param snapshot.',
    )
    scenario_name_arg = DeclareLaunchArgument(
        'scenario_name', default_value='unknown',
        description='Short scenario name written into the JSONL filename.',
    )
    runs_dir_arg = DeclareLaunchArgument(
        'runs_dir', default_value=os.path.expanduser('~/.drone_rescue/runs'),
        description='Where mission_recorder writes JSONLs.',
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description=(
            'Run Gazebo server-only (no gz-sim GUI window). Forwarded '
            'to multi_drone_simulation.launch.py. The operator GUI is '
            'the dashboard / Mission Control; the gz-sim client window '
            'is redundant and its orphan-on-SIGTERM children leak '
            'processes. Set false for debugging the physics scene '
            'directly in gz-sim.'
        ),
        choices=['true', 'false'],
    )

    # Include the full multi-drone simulation
    # This brings up: Gazebo, drones, coordination nodes, bridges, and visualization
    # (including telemetry_overlay via visualization.launch.py).
    # When the dashboard is enabled it embeds RViz itself, so we forward
    # use_rviz=false in that case to avoid two RViz instances.
    rviz_in_sim = PythonExpression([
        '"', LaunchConfiguration('use_rviz'),
        '" if "', LaunchConfiguration('dashboard'), '" == "false" else "false"',
    ])
    multi_drone_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drone_rescue_bringup, 'launch', 'multi_drone_simulation.launch.py')
        ),
        launch_arguments={
            'num_drones': LaunchConfiguration('num_drones'),
            'use_rviz': rviz_in_sim,
            'record_bag': LaunchConfiguration('record_bag'),
            'coverage_pattern': LaunchConfiguration('coverage_pattern'),
            'allocation_strategy': LaunchConfiguration('allocation_strategy'),
            'seed': LaunchConfiguration('seed'),
            'record_run': LaunchConfiguration('record_run'),
            'scenario_yaml': LaunchConfiguration('scenario_yaml'),
            'scenario_name': LaunchConfiguration('scenario_name'),
            'runs_dir': LaunchConfiguration('runs_dir'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    dashboard_pkg = get_package_share_directory('drone_rescue_dashboard')
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dashboard_pkg, 'launch', 'dashboard.launch.py'),
        ),
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    # Camera director node - executes scripted shot sequence
    # Delayed start to allow Gazebo GUI to initialize and drones to spawn
    camera_director = Node(
        package='drone_rescue_coordination',
        executable='camera_director',
        name='camera_director',
        parameters=[{
            'use_sim_time': True,
            'shot_config': PathJoinSubstitution([
                FindPackageShare('drone_rescue_bringup'),
                'config', 'camera_shots.yaml'
            ]),
            'drone_names': ['drone1', 'drone2', 'drone3', 'drone4'],
            'enable_camera_control': LaunchConfiguration('enable_camera'),
            'demo_timeout_sim_seconds': ParameterValue(LaunchConfiguration('demo_timeout'), value_type=float),
            'coverage_completion_threshold': ParameterValue(LaunchConfiguration('coverage_completion'), value_type=float),
            'coverage_survey_threshold': ParameterValue(LaunchConfiguration('coverage_survey'), value_type=float),
        }],
        output='screen',
    )

    # Delay camera director start by 12 seconds
    # Rationale:
    # - Gazebo launches at t=0
    # - Drones spawn at t=0-1s
    # - Controllers start at t=5s
    # - Lifecycle manager at t=8s
    # - Camera director at t=12s (GUI ready, drones initializing)
    delayed_camera_director = TimerAction(
        period=12.0,
        actions=[camera_director],
    )

    return LaunchDescription([
        # Launch arguments
        num_drones_arg,
        use_rviz_arg,
        enable_camera_arg,
        record_bag_arg,
        demo_timeout_arg,
        coverage_completion_arg,
        coverage_survey_arg,
        coverage_pattern_arg,
        allocation_strategy_arg,
        seed_arg,
        dashboard_arg,
        record_run_arg,
        scenario_yaml_arg,
        scenario_name_arg,
        runs_dir_arg,
        headless_arg,

        # Multi-drone simulation (includes everything except camera director)
        multi_drone_simulation,

        # Unified rqt dashboard (when dashboard:=true).
        dashboard_launch,

        # Camera director (delayed to allow Gazebo and drones to initialize)
        delayed_camera_director,
    ])
