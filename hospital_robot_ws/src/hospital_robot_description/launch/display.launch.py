#!/usr/bin/env python3
"""
Launch file: Display hospital TurtleBot4 in RViz2
Used for URDF verification and visualization.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('hospital_robot_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'hospital_turtlebot4.urdf.xacro')

    use_gui = LaunchConfiguration('use_gui', default='true')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Launch joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
