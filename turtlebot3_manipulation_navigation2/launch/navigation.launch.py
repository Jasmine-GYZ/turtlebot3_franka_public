#!/usr/bin/env python3
"""
导航启动文件
用法:
  ros2 launch turtlebot3_manipulation_gazebo navigation.launch.py
  ros2 launch turtlebot3_manipulation_gazebo navigation.launch.py start_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # ── 默认地图：map ──────────────────────────────
    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=PathJoinSubstitution([
            FindPackageShare('turtlebot3_manipulation_navigation2'),
            'map',
            'map.yaml',
        ]))

    # ── 默认参数：turtlebot3_use_sim_time（仿真已配好）──────────
    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([
            FindPackageShare('turtlebot3_manipulation_navigation2'),
            'param',
            'turtlebot3_use_sim_time.yaml',
        ]))

    nav2_launch_file_dir = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_manipulation_navigation2'),
        'rviz',
        'navigation2.rviz',
    ])

    default_bt_xml_filename = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz', default_value='true',
            description='是否启动 RViz'),

        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='使用仿真时钟'),

        DeclareLaunchArgument(
            'map_yaml_file', default_value=map_yaml_file,
            description='地图 yaml 文件路径'),

        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='Nav2 参数文件路径'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='自动启动 Nav2 栈'),

        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='是否使用 composed bringup'),

        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            description='节点崩溃时是否自动重启'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim,
                'params_file': params_file,
                'default_bt_xml_filename': default_bt_xml_filename,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)),

        # Nav2 velocity_smoother 输出 /cmd_vel，而 diff_drive_controller
        # 订阅的是命名空间下的 /diff_drive_controller/cmd_vel_unstamped
        # （use_stamped_vel: false 时话题名为 ~/cmd_vel_unstamped）。
        # 用自定义 relay 节点桥接，否则机器人导航时不移动。
        Node(
            package='turtlebot3_manipulation_navigation2',
            executable='cmd_vel_relay.py',
            name='cmd_vel_relay',
            output='screen'),
    ])
