from setuptools import find_packages, setup

package_name = 'drone_rescue_viz'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Visualization nodes for RViz',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pheromone_visualizer = drone_rescue_viz.pheromone_visualizer:main',
            'drone_trails = drone_rescue_viz.drone_trails:main',
            'coverage_visualizer = drone_rescue_viz.coverage_visualizer:main',
            'victim_visualizer = drone_rescue_viz.victim_visualizer:main',
            'telemetry_overlay = drone_rescue_viz.telemetry_overlay:main',
        ],
    },
)
