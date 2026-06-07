#!/usr/bin/env python3
"""
Multi-Robot Hospital Launch
=============================
Launches 3 TurtleBot4 hospital robots with:
  - Shared hospital Gazebo world
  - Namespaced Nav2 stacks (robot_1, robot_2, robot_3)
  - Namespaced SLAM Toolbox with shared map
  - Central multi-robot coordinator (Hungarian algorithm)
  - Per-robot task managers
  - Per-robot obstacle trackers
  - RViz2 multi-robot view

Usage:
  ros2 launch hospital_bringup hospital_multi_robot.launch.py

Spawn positions:
  robot_1: (0,  0)   - Main corridor center
  robot_2: (15, 0)   - East corridor
  robot_3: (-20, 0)  - West corridor
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_desc    = get_package_share_directory('hospital_robot_description')
    pkg_gazebo  = get_package_share_directory('hospital_gazebo')
    pkg_nav     = get_package_share_directory('hospital_navigation')
    pkg_bringup = get_package_share_directory('hospital_bringup')
    pkg_gz_sim  = get_package_share_directory('ros_gz_sim')

    xacro_file    = os.path.join(pkg_desc, 'urdf', 'hospital_turtlebot4.urdf.xacro')
    nav2_params   = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    slam_params   = os.path.join(pkg_nav, 'config', 'slam_toolbox_params.yaml')
    world_file    = os.path.join(pkg_gazebo, 'worlds', 'hospital.world')
    rviz_config   = os.path.join(pkg_bringup, 'config', 'hospital_multi_rviz.rviz')

    # ==================== ARGUMENTS =======================
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_rviz     = LaunchConfiguration('use_rviz')

    # ==================== ENV =============================
    set_display = SetEnvironmentVariable(name='DISPLAY', value=':0')

    # ==================== SHARED GAZEBO ===================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ==================== GZ BRIDGE (shared topics) =======
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge_shared',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]
    )

    # ==================== ROBOT FACTORY ===================
    robot_configs = [
        {'id': 'robot_1', 'x':   0.0, 'y':  0.0, 'yaw': 0.0},
        {'id': 'robot_2', 'x':  15.0, 'y':  0.0, 'yaw': 3.14},
        {'id': 'robot_3', 'x': -20.0, 'y':  0.0, 'yaw': 0.0},
    ]

    robot_actions = []
    spawn_delay   = 3.0  # seconds between spawns to avoid physics conflicts

    for i, cfg in enumerate(robot_configs):
        ns      = cfg['id']
        robot_description = Command([
            'xacro ', xacro_file,
            ' namespace:=', ns,
            ' robot_name:=', ns,
        ])

        # Per-robot state publisher
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'publish_frequency': 30.0,
                'frame_prefix': ns + '/',
            }]
        )

        # Per-robot gz bridge
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'gz_bridge_{ns}',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ]
        )

        # Spawn robot
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{ns}',
            output='screen',
            arguments=[
                '-name',   ns,
                '-topic',  f'/{ns}/robot_description',
                '-x',      str(cfg['x']),
                '-y',      str(cfg['y']),
                '-z',      '0.05',
                '-Y',      str(cfg['yaw']),
            ]
        )

        # Per-robot Nav2 (Jazzy: use_namespace is ignored, must wrap in PushRosNamespace)
        nav2 = GroupAction([
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ),
                launch_arguments={
                    'use_sim_time':    'true',
                    'params_file':     nav2_params,
                    'namespace':       ns,
                    'autostart':       'true',
                    'use_composition': 'false',
                    'log_level':       'warn',
                }.items()
            )
        ])

        # Per-robot SLAM
        slam = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[
                slam_params,
                {
                    'use_sim_time': True,
                    'odom_frame':   f'{ns}/odom',
                    'base_frame':   f'{ns}/base_footprint',
                    'scan_topic':   f'/{ns}/scan',
                    'map_frame':    'map',
                }
            ]
        )

        # Per-robot obstacle tracker
        tracker = Node(
            package='hospital_mission',
            executable='obstacle_tracker',
            name='obstacle_tracker',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'scan_topic': f'/{ns}/scan',
                'odom_topic': f'/{ns}/odom',
            }]
        )

        # Per-robot task manager
        task_mgr = Node(
            package='hospital_mission',
            executable='task_manager',
            name='task_manager',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_id':    ns,
                'robot_namespace': ns,
            }]
        )

        delay_base = spawn_delay * i

        robot_actions.extend([
            TimerAction(period=delay_base + 2.0,  actions=[rsp]),
            TimerAction(period=delay_base + 2.0,  actions=[bridge]),
            TimerAction(period=delay_base + 4.0,  actions=[spawn]),
            TimerAction(period=delay_base + 10.0, actions=[slam]),
            TimerAction(period=delay_base + 12.0, actions=[nav2]),
            TimerAction(period=delay_base + 15.0, actions=[tracker]),
            TimerAction(period=delay_base + 16.0, actions=[task_mgr]),
        ])

    # ==================== MULTI-ROBOT COORDINATOR =========
    coordinator = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='hospital_mission',
                executable='multi_robot_coordinator',
                name='multi_robot_coordinator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # ==================== RVIZ2 (multi-robot view) ========
    rviz2 = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(use_rviz)
            )
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        set_display,
        gazebo,
        gz_bridge,
        *robot_actions,
        coordinator,
        rviz2,
    ])
