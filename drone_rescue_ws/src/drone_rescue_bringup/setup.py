import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_rescue_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) +
            glob(os.path.join('config', '*.xml'))),
        # Install scenario YAMLs (Mission Control reads these from share/)
        (os.path.join('share', package_name, 'config', 'scenarios'),
            glob(os.path.join('config', 'scenarios', '*.yaml'))),
        # Install rviz configs
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
        # Install integration tests
        (os.path.join('share', package_name, 'test', 'integration'),
            glob(os.path.join('test', 'integration', '*.py'))),
        # Install scripts
        (os.path.join('share', package_name, 'scripts'),
            glob(os.path.join('scripts', '*.sh'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Launch files and configurations for drone rescue simulation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
