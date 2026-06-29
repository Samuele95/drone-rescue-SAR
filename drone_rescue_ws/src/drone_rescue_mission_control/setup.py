from setuptools import find_packages, setup

package_name = 'drone_rescue_mission_control'

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
    description=(
        'PyQt5 launcher + mission recorder + multi-run analytics for the '
        'drone-rescue SAR sim.'
    ),
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mission_control = drone_rescue_mission_control.mission_control_app:main',
            'mission_recorder = drone_rescue_mission_control.mission_recorder:main',
            # V5: headless batch sweep runner.
            'bench = drone_rescue_mission_control.bench:main',
            # V5: PDF report generator (per-run or per-sweep).
            'report = drone_rescue_mission_control.report:main',
        ],
    },
)
