from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hospital_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hospital Robot',
    maintainer_email='hospital@robot.dev',
    description='Autonomous hospital delivery robot system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager     = hospital_robot.mission_manager:main',
            'fleet_coordinator   = hospital_robot.fleet_coordinator:main',
            'obstacle_tracker    = hospital_robot.obstacle_tracker:main',
            'frontier_explorer   = hospital_robot.frontier_explorer:main',
            'dynamic_obstacles   = hospital_robot.dynamic_obstacles:main',
            'health_monitor      = hospital_robot.health_monitor:main',
        ],
    },
)
