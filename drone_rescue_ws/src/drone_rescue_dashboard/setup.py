from setuptools import find_packages, setup

package_name = 'drone_rescue_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/dashboard.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description=(
        'Multipage PyQt5 mission dashboard — Overview, All Cameras, '
        'per-drone tabs, full mission log.'
    ),
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'dashboard = drone_rescue_dashboard.dashboard_app:main',
        ],
    },
)
