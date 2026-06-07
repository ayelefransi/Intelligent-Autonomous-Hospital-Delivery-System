from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hospital_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hospital Robot Team',
    maintainer_email='hospital_robot@example.com',
    description='Hospital delivery mission management system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager        = hospital_mission.task_manager:main',
            'multi_robot_coordinator = hospital_mission.multi_robot_coordinator:main',
            'obstacle_tracker    = hospital_mission.obstacle_tracker:main',
            'frontier_explorer   = hospital_mission.frontier_explorer:main',
        ],
    },
)
