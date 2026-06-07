#!/usr/bin/env python3
"""
hospital_slam.launch.py
========================
Full single-robot system in SLAM mode.
Spawns TurtleBot3 Waffle in the AWS RoboMaker hospital world.

Launch sequence (timed to prevent race conditions):
  t=0   Gazebo Harmonic + hospital world
  t=3   gz->ROS bridge (clock, scan, odom, cmd_vel, tf, imu)
  t=5   robot_state_publisher (custom URDF with tray + gripper)
  t=12  spawn robot via ros_gz_sim create (ExecuteProcess with SDF file)
  t=15  SLAM Toolbox (async online)
  t=30  Nav2 navigation stack (long delay — SLAM needs time to publish map frame)
  t=21  Obstacle tracker
  t=28  Dynamic obstacles
  t=32  Health monitor
  t=33  Mission manager (queues demo deliveries)
  t=35  Frontier explorer
  t=14  RViz2

Usage:
  ros2 launch hospital_robot hospital_slam.launch.py
  ros2 launch hospital_robot hospital_slam.launch.py use_rviz:=false
  ros2 launch hospital_robot hospital_slam.launch.py x_pose:=2.0 y_pose:=-3.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, SetEnvironmentVariable, ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_hr     = get_package_share_directory('hospital_robot')
    pkg_hwb    = get_package_share_directory('hospital_world_bridge')
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2   = get_package_share_directory('nav2_bringup')
    pkg_gz     = get_package_share_directory('ros_gz_sim')

    # ── Arguments ────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_rviz',   default_value='true'),
        DeclareLaunchArgument('use_explore',default_value='true'),
        DeclareLaunchArgument('x_pose',     default_value='-3.0'),
        DeclareLaunchArgument('y_pose',     default_value='2.0'),
        DeclareLaunchArgument('yaw',        default_value='0.0'),
    ]
    use_rviz    = LaunchConfiguration('use_rviz')
    use_explore = LaunchConfiguration('use_explore')
    x_pose      = LaunchConfiguration('x_pose')
    y_pose      = LaunchConfiguration('y_pose')
    yaw         = LaunchConfiguration('yaw')

    # ── Config paths ──────────────────────────────────────────────────────────
    nav2_params_file  = os.path.join(pkg_hr, 'config', 'nav2_params.yaml')
    slam_params_file  = os.path.join(pkg_hr, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file  = os.path.join(pkg_hr, 'config', 'hospital.rviz')
    bridge_config_file = os.path.join(pkg_hr, 'config', 'gz_bridge.yaml')

    # World: AWS RoboMaker Hospital World (only)
    hospital_world = os.path.join(pkg_hwb, 'worlds', 'hospital.world')

    # TurtleBot3 SDF model (with gazebo plugins — NOT -topic)
    tb3_model_sdf = os.path.join(pkg_tb3_gz, 'models', 'turtlebot3_waffle', 'model.sdf')

    # ── Build custom URDF: TB3 Waffle base + hospital addons ──────────────
    # Read stock TB3 URDF, strip closing </robot>, append addon links,
    # then close.  No xacro — avoids $(find) resolution and nested-robot bugs.
    tb3_urdf_path = os.path.join(pkg_tb3_gz, 'urdf', 'turtlebot3_waffle.urdf')
    addon_urdf_path = os.path.join(pkg_hr, 'urdf', 'hospital_addons.urdf')
    with open(tb3_urdf_path, 'r') as f:
        tb3_content = f.read()
    with open(addon_urdf_path, 'r') as f:
        addon_content = f.read()
    # Strip XML declaration from addon if present
    addon_content = addon_content.replace('<?xml version="1.0"?>', '')
    # Find the closing </robot> tag and splice in addon content before it
    robot_close = tb3_content.rfind('</robot>')
    if robot_close > 0:
        robot_desc = tb3_content[:robot_close] + addon_content + '\n</robot>'
    else:
        robot_desc = tb3_content  # fallback

    # ── WSLg display ─────────────────────────────────────────────────────────
    set_display = SetEnvironmentVariable('DISPLAY', ':0')
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # GZ resource path: preserve existing + add hospital + turtlebot3 paths
    import os as _os
    _existing = _os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    _needed = (
        os.path.join(pkg_hwb, 'models') + ':' +
        os.path.join(pkg_hwb, 'fuel_models') + ':' +
        os.path.join(pkg_tb3_gz, 'models') + ':' +
        os.path.dirname(pkg_tb3_gz)
    )
    _combined = (_needed + ':' + _existing) if _existing else _needed
    set_gz_resources = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', _combined)

    # ── 1. Gazebo Harmonic ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', hospital_world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── 2. GZ <-> ROS bridge (using YAML config — model-specific TF) ──
    gz_bridge = TimerAction(period=3.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['--ros-args', '-p', f'config_file:={bridge_config_file}'],
        )
    ])

    # ── 3. Robot state publisher (custom URDF: hospital_robot with tray+gripper) ─
    rsp = TimerAction(period=5.0, actions=[
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
                'publish_frequency': 30.0,
            }],
        )
    ])

    # ── 4. Spawn TurtleBot3 (ExecuteProcess with SDF file — preserves plugins) ─
    spawn_robot = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-world', 'hospital',
                 '-name',  'turtlebot3_waffle',
                 '-file', tb3_model_sdf,
                 '-x', x_pose,
                 '-y', y_pose,
                 '-z', '0.01',
                 '-Y', yaw],
            output='screen',
        )
    ])

    # ── 4b. Pre-spawn red payload box at pharmacy (initially exists, robot picks up later) ─
    spawn_pharmacy_payload = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-world', 'hospital',
                 '-name',  'payload_pharmacy',
                 '-file', os.path.join(pkg_hwb, 'models', 'payload_box', 'model.sdf'),
                 '-x', '9.0',
                 '-y', '10.0',
                 '-z', '0.85'],
            output='screen',
        )
    ])

    # ── 5. SLAM Toolbox ───────────────────────────────────────────────────────
    slam = TimerAction(period=15.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'),
                           'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'true',
            }.items()
        )
    ])

    # ── 6. Nav2 ───────────────────────────────────────────────────────────────
    nav2 = TimerAction(period=30.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time':    'True',
                'params_file':     nav2_params_file,
                'autostart':       'True',
                'use_composition': 'False',
                'use_velocity_smoother': 'False',
                'use_collision_monitor': 'False',
            }.items()
        )
    ])

    # ── 7. Obstacle tracker ───────────────────────────────────────────────────
    tracker = TimerAction(period=21.0, actions=[
        Node(
            package='hospital_robot',
            executable='obstacle_tracker',
            name='obstacle_tracker',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── 7b. Health monitor ────────────────────────────────────────────────────
    health = TimerAction(period=32.0, actions=[
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

    # ── 8. Mission manager ────────────────────────────────────────────────────
    mission = TimerAction(period=33.0, actions=[
        Node(
            package='hospital_robot',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_id': 'robot_1',
                'robot_spawn_x': x_pose,
                'robot_spawn_y': y_pose,
            }],
        )
    ])

    # ── 9. Dynamic obstacles ──────────────────────────────────────────────────
    dynamic_obs = TimerAction(period=28.0, actions=[
        Node(
            package='hospital_robot',
            executable='dynamic_obstacles',
            name='dynamic_obstacles',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── 10. Frontier explorer ──────────────────────────────────────────────────
    explorer = TimerAction(period=35.0, actions=[
        Node(
            package='hospital_robot',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_explore),
        )
    ])

    # ── 10. RViz2 ─────────────────────────────────────────────────────────────
    rviz = TimerAction(period=14.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        )
    ])

    # ── 11. Auto-send delivery tasks (after mission_manager + Nav2 are ready) ──
    send_tasks = TimerAction(period=60.0, actions=[
        ExecuteProcess(
            cmd=['python3', '/home/robot/hospital_ws/send_deliveries.py'],
            output='screen',
        )
    ])

    return LaunchDescription([
        *args,
        set_display,
        set_tb3_model,
        set_gz_resources,
        gazebo,
        gz_bridge,
        rsp,
        spawn_robot,
        spawn_pharmacy_payload,
        slam,
        nav2,
        tracker,
        health,
        mission,
        dynamic_obs,
        explorer,
        rviz,
        send_tasks,
    ])
