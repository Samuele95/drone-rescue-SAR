from setuptools import find_packages, setup

package_name = 'drone_rescue_coordination'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Pheromone coordination and drone control algorithms',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_controller = drone_rescue_coordination.drone_controller:main',
            'battery_monitor = drone_rescue_coordination.battery_monitor:main',
            'flight_test = drone_rescue_coordination.flight_test:main',
            'environment_monitor = drone_rescue_coordination.environment_monitor:main',
            'zone_manager = drone_rescue_coordination.zone_manager:main',
            'pheromone_server = drone_rescue_coordination.pheromone_server:main',
            'coverage_tracker = drone_rescue_coordination.coverage_tracker:main',
            'victim_detector = drone_rescue_coordination.victim_detector:main',
            'sensor_degradation = drone_rescue_coordination.sensor_degradation:main',
            'readiness_coordinator = drone_rescue_coordination.readiness_coordinator:main',
            'lifecycle_manager = drone_rescue_coordination.lifecycle_manager:main',
            'camera_director = drone_rescue_coordination.camera_director:main',
            # SAR redesign: Mediator + Saga + Behavior Tree architecture.
            'mission_manager = drone_rescue_coordination.mission_manager:main',
            # Feature-flagged post-deconstruction node.
            # USE_LEGACY_MISSION_MANAGER=0 to exercise (currently falls back
            # to legacy with a warning until the saga migration is complete).
            'mission_manager_v2 = drone_rescue_coordination.mission_manager_node:main',
            'detection_filter = drone_rescue_coordination.detection_filter:main',
            'drone_executor = drone_rescue_coordination.drone_executor:main',
            # Feature-flagged post-deconstruction node.
            # USE_LEGACY_DRONE_EXECUTOR=0 to exercise (currently falls back
            # to legacy with a warning until the BT-action cascade is done).
            'drone_executor_v2 = drone_rescue_coordination.drone_executor_node:main',
            # Per-drone health watchdog with anomaly fusion.
            'drone_health_monitor = drone_rescue_coordination.drone_health_monitor:main',
        ],
    },
)
