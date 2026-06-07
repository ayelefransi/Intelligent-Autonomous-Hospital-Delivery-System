#!/usr/bin/env python3
"""
hospital_multi.launch.py
=========================
3x TurtleBot3 Waffle robots in the AWS hospital world.
Each robot gets its own:
  - Namespaced gz bridge
  - robot_state_publisher
  - SLAM Toolbox
  - Nav2 stack
  - obstacle_tracker
  - mission_manager

Central:
  - fleet_coordinator (Hungarian algorithm)
  - RViz2 (multi-robot view)

Namespaces: robot_1, robot_2, robot_3

Spawn positions (clear corridor areas):
  robot_1:  (-3.5,  1.0, 0.0)   reception area
  robot_2:  ( 3.5,  1.0, 3.14)  opposite reception
  robot_3:  (-3.5, -3.0, 0.0)   south corridor

Usage:
  ros2 launch hospital_robot hospital_multi.launch.py
  ros2 launch hospital_robot hospital_multi.launch.py use_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, SetEnvironmentVariable, GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    pkg_hr     = get_package_share_directory('hospital_robot')
    pkg_hwb    = get_package_share_directory('hospital_world_bridge')
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2   = get_package_share_directory('nav2_bringup')
    pkg_gz     = get_package_share_directory('ros_gz_sim')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_rviz     = LaunchConfiguration('use_rviz')

    nav2_params = os.path.join(pkg_hr, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_hr, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg_hr, 'config', 'hospital.rviz')
    world_file  = os.path.join(pkg_hwb, 'worlds', 'hospital.world')

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

    # ── Shared Gazebo ─────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── Shared clock bridge ───────────────────────────────────────────────────
    clock_bridge = TimerAction(period=3.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Per-robot factory ─────────────────────────────────────────────────────
    robot_configs = [
        {'id': 'robot_1', 'x': -3.5, 'y':  1.0, 'yaw': 0.0,  'delay': 0.0},
        {'id': 'robot_2', 'x':  3.5, 'y':  1.0, 'yaw': 3.14, 'delay': 3.0},
        {'id': 'robot_3', 'x': -3.5, 'y': -3.0, 'yaw': 0.0,  'delay': 6.0},
    ]

    all_actions = []

    for cfg in robot_configs:
        ns    = cfg['id']
        base  = cfg['delay']

        # Per-robot gz topic bridge
        bridge = TimerAction(period=base + 4.0, actions=[
            Node(
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
                ],
            )
        ])

        # robot_state_publisher (namespaced)
        rsp = TimerAction(period=base + 5.0, actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=ns,
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True,
                    'frame_prefix': f'{ns}/',
                }],
            )
        ])

        # Spawn robot
        spawn = TimerAction(period=base + 6.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{ns}',
                output='screen',
                arguments=[
                    '-name',  ns,
                    '-topic', f'/{ns}/robot_description',
                    '-x',     str(cfg['x']),
                    '-y',     str(cfg['y']),
                    '-z',     '0.01',
                    '-Y',     str(cfg['yaw']),
                ],
            )
        ])

        # SLAM Toolbox (namespaced)
        slam = TimerAction(period=base + 10.0, actions=[
            Node(
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
                        'map_frame':    'map',
                        'scan_topic':   f'/{ns}/scan',
                    },
                ],
            )
        ])

        # Nav2 (namespaced)
        nav2 = TimerAction(period=base + 13.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time':    'True',
                    'params_file':     nav2_params,
                    'namespace':       ns,
                    'use_namespace':   'True',
                    'autostart':       'True',
                    'use_composition': 'False',
                    'log_level':       'warn',
                }.items()
            )
        ])

        # Obstacle tracker (namespaced)
        tracker = TimerAction(period=base + 17.0, actions=[
            Node(
                package='hospital_robot',
                executable='obstacle_tracker',
                name='obstacle_tracker',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'scan_topic':   f'/{ns}/scan',
                }],
            )
        ])

        # Health monitor (namespaced)
        health = TimerAction(period=base + 18.0, actions=[
            Node(
                package='hospital_robot',
                executable='health_monitor',
                name='health_monitor',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_id':     ns,
                    'robot_ns':     ns,
                }],
            )
        ])

        # Mission manager (namespaced)
        mission = TimerAction(period=base + 19.0, actions=[
            Node(
                package='hospital_robot',
                executable='mission_manager',
                name='mission_manager',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_id':     ns,
                    'robot_ns':     ns,
                }],
            )
        ])

        all_actions.extend([bridge, rsp, spawn, slam, nav2, tracker, health, mission])

    # ── Fleet coordinator ─────────────────────────────────────────────────────
    coordinator = TimerAction(period=25.0, actions=[
        Node(
            package='hospital_robot',
            executable='fleet_coordinator',
            name='fleet_coordinator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz = TimerAction(period=8.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        )
    ])

    return LaunchDescription([
        use_rviz_arg,
        set_display,
        set_tb3,
        set_gz_resources,
        gazebo,
        clock_bridge,
        *all_actions,
        coordinator,
        rviz,
    ])
