#!/usr/bin/env python3
"""
hospital_nav.launch.py
=======================
Single robot, pre-built map, AMCL localization.
Use after saving a map with:
  ros2 run nav2_map_server map_saver_cli -f ~/hospital_map

Usage:
  ros2 launch hospital_robot hospital_nav.launch.py \
      map:=$HOME/hospital_map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_hr     = get_package_share_directory('hospital_robot')
    pkg_hwb    = get_package_share_directory('hospital_world_bridge')
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2   = get_package_share_directory('nav2_bringup')
    pkg_gz     = get_package_share_directory('ros_gz_sim')

    args = [
        DeclareLaunchArgument('map',      default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('x_pose',   default_value='-3.5'),
        DeclareLaunchArgument('y_pose',   default_value='1.0'),
        DeclareLaunchArgument('yaw',      default_value='0.0'),
    ]
    map_path = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose   = LaunchConfiguration('x_pose')
    y_pose   = LaunchConfiguration('y_pose')
    yaw      = LaunchConfiguration('yaw')

    nav2_params  = os.path.join(pkg_hr,  'config', 'nav2_params.yaml')
    rviz_config  = os.path.join(pkg_hr,  'config', 'hospital.rviz')
    world_file   = os.path.join(pkg_hwb, 'worlds', 'hospital.world')

    tb3_urdf = os.path.join(pkg_tb3_gz, 'urdf', 'turtlebot3_waffle.urdf')
    with open(tb3_urdf, 'r') as f:
        robot_desc = f.read()

    set_display      = SetEnvironmentVariable('DISPLAY', ':0')
    set_tb3          = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    set_gz_resources = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_hwb, 'models') + ':' +
        os.path.join(pkg_tb3_gz, 'models')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items()
    )

    gz_bridge = TimerAction(period=3.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
        )
    ])

    rsp = TimerAction(period=5.0, actions=[
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }],
        )
    ])

    spawn = TimerAction(period=6.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'turtlebot3_waffle',
                '-topic', 'robot_description',
                '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw,
            ],
        )
    ])

    # Map server + AMCL (not SLAM)
    map_server = TimerAction(period=9.0, actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_path,
            }],
        )
    ])

    amcl = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': True}],
        )
    ])

    nav2 = TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time':    'True',
                'params_file':     nav2_params,
                'autostart':       'True',
                'use_composition': 'False',
            }.items()
        )
    ])

    tracker = TimerAction(period=15.0, actions=[
        Node(
            package='hospital_robot',
            executable='obstacle_tracker',
            name='obstacle_tracker',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    health = TimerAction(period=16.0, actions=[
        Node(
            package='hospital_robot',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_id': 'robot_1',
            }],
        )
    ])

    mission = TimerAction(period=17.0, actions=[
        Node(
            package='hospital_robot',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_id': 'robot_1'}],
        )
    ])

    rviz = TimerAction(period=8.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        )
    ])

    return LaunchDescription([
        *args,
        set_display, set_tb3, set_gz_resources,
        gazebo, gz_bridge, rsp, spawn,
        map_server, amcl, nav2,
        tracker, health, mission, rviz,
    ])
