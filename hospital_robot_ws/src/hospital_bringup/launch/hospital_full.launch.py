#!/usr/bin/env python3
"""
Hospital Robot Full System Launch
===================================
Single TurtleBot4 hospital delivery robot.
Launches:
  - Gazebo Harmonic with hospital world
  - Robot spawner (from Fuel or local URDF)
  - robot_state_publisher
  - Nav2 stack (AMCL + Nav2)
  - SLAM Toolbox (online async mode)
  - Dynamic obstacle tracker
  - Task manager
  - Frontier explorer
  - RViz2

Usage:
  ros2 launch hospital_bringup hospital_full.launch.py
  ros2 launch hospital_bringup hospital_full.launch.py mode:=slam
  ros2 launch hospital_bringup hospital_full.launch.py mode:=nav map:=/path/to/map.yaml

WSL2 + WSLg: GUI renders via WSLg automatically.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ==================== PACKAGE PATHS ===================
    pkg_desc        = get_package_share_directory('hospital_robot_description')
    pkg_gazebo      = get_package_share_directory('hospital_gazebo')
    pkg_nav         = get_package_share_directory('hospital_navigation')
    pkg_bringup     = get_package_share_directory('hospital_bringup')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # ==================== ARGUMENTS =======================
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Navigation mode: slam (builds map) or nav (uses existing map)'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map YAML (required for nav mode)'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='hospital_robot',
        description='Robot name for spawning'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS 2 namespace'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    use_exploration_arg = DeclareLaunchArgument(
        'use_exploration',
        default_value='true',
        description='Launch frontier explorer'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # ==================== CONFIGURATIONS ==================
    mode          = LaunchConfiguration('mode')
    map_path      = LaunchConfiguration('map')
    robot_name    = LaunchConfiguration('robot_name')
    namespace     = LaunchConfiguration('namespace')
    use_rviz      = LaunchConfiguration('use_rviz')
    use_exploration = LaunchConfiguration('use_exploration')
    spawn_x       = LaunchConfiguration('spawn_x')
    spawn_y       = LaunchConfiguration('spawn_y')
    spawn_yaw     = LaunchConfiguration('spawn_yaw')

    xacro_file = os.path.join(
        pkg_desc, 'urdf', 'hospital_turtlebot4.urdf.xacro'
    )
    robot_description = Command(['xacro ', xacro_file])

    nav2_params_file = os.path.join(
        pkg_nav, 'config', 'nav2_params.yaml'
    )
    slam_params_file = os.path.join(
        pkg_nav, 'config', 'slam_toolbox_params.yaml'
    )
    rviz_config_file = os.path.join(
        pkg_bringup, 'config', 'hospital_rviz.rviz'
    )
    world_file = os.path.join(
        pkg_gazebo, 'worlds', 'hospital.world'
    )

    # ==================== WSLg ENV ========================
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=':0'
    )

    # ==================== GAZEBO ==========================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ==================== ROBOT STATE PUBLISHER ===========
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 30.0,
        }]
    )

    # ==================== SPAWN ROBOT =====================
    # Bridge gz -> ROS before spawning
    gz_bridge = Node(
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
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hospital_robot',
        output='screen',
        arguments=[
            '-name',   robot_name,
            '-topic',  'robot_description',
            '-x',      spawn_x,
            '-y',      spawn_y,
            '-z',      '0.05',
            '-Y',      spawn_yaw,
        ]
    )

    # ==================== SLAM TOOLBOX (slam mode) ========
    slam_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': True}
                ],
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'slam'"])
                )
            )
        ]
    )

    # ==================== MAP SERVER (nav mode) ===========
    map_server = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': map_path,
                }],
                condition=IfCondition(
                    PythonExpression(["'", mode, "' == 'nav'"])
                )
            )
        ]
    )

    # ==================== NAV2 STACK ======================
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ),
                launch_arguments={
                    'use_sim_time':     'true',
                    'params_file':      nav2_params_file,
                    'autostart':        'true',
                    'use_composition':  'false',
                }.items()
            )
        ]
    )

    # ==================== OBSTACLE TRACKER ================
    obstacle_tracker = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='hospital_mission',
                executable='obstacle_tracker',
                name='obstacle_tracker',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # ==================== TASK MANAGER ====================
    task_manager = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='hospital_mission',
                executable='task_manager',
                name='task_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_id': robot_name,
                }]
            )
        ]
    )

    # ==================== FRONTIER EXPLORER ===============
    frontier_explorer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='hospital_mission',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(use_exploration)
            )
        ]
    )

    # ==================== RVIZ2 ==========================
    rviz2 = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(use_rviz)
            )
        ]
    )

    # ==================== TELEOP (optional) ===============
    teleop_info = ExecuteProcess(
        cmd=['echo', '\n[INFO] Teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        mode_arg, map_arg, robot_name_arg, namespace_arg,
        use_rviz_arg, use_exploration_arg,
        spawn_x_arg, spawn_y_arg, spawn_yaw_arg,

        # Environment
        set_display,

        # Simulation
        gazebo,
        robot_state_publisher,
        gz_bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),

        # SLAM or Map server
        slam_node,
        map_server,

        # Navigation
        nav2,

        # Mission
        obstacle_tracker,
        task_manager,
        frontier_explorer,

        # Visualization
        rviz2,
        teleop_info,
    ])
