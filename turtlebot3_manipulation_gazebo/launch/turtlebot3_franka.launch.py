#!/usr/bin/env python3
"""
Fortress 仿真启动文件

一键启动 Gazebo Sim + TurtleBot3(带臂) + ros2_control + 传感器桥接
用法:
  ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
  ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py rviz:=true
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("turtlebot3_manipulation_gazebo")
    wpr_share = get_package_share_directory("wpr_simulation_ros2")

    rviz = LaunchConfiguration("rviz", default="false")
    world_file = os.path.join(wpr_share, "worlds", "official-ros2.world")

    # ── 1. xacro → URDF ─────────────────────────────────────────
    xacro_path = os.path.join(pkg_share, "urdf",
                              "turtlebot3_manipulation.urdf.xacro")
    doc = xacro.process_file(xacro_path, mappings={"use_sim": "true"})
    robot_description_xml = doc.toxml()

    # ── 2. 资源路径 ──────────────────────────────────────────────
    # model://package_name/... URI 解析：Gazebo 在 IGN_GAZEBO_RESOURCE_PATH
    # 的每个目录下查找 package_name/ 子目录。必须包含 PACKAGE 的父目录。
    # 同时把 /opt/ros/humble/lib 加入系统插件路径，让 Gazebo 能找到
    # libgz_ros2_control-system.so。
    # model://franka_description/... 需要 franka_description 的父目录
    franka_share = get_package_share_directory("franka_description")
    resource_path = os.pathsep.join([
        os.path.join(pkg_share, "models"),
        os.path.join(wpr_share, "models"),
        os.path.dirname(pkg_share),       # model://turtlebot3_manipulation_gazebo/meshes/...
        os.path.dirname(franka_share),    # model://franka_description/meshes/...
        "/opt/ros/humble/lib",            # libgz_ros2_control-system.so
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    ])

    # ── 3. 节点 ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml,
                     "use_sim_time": True}],
    )

    # Humble = Fortress = ign gazebo（不是 gz sim）
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", world_file, "-r"],
        output="screen",
    )

    # 从 /robot_description 话题 spawn 机器人
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description",
                   "-name", "turtlebot3",
                   "-x", "-5.30",
                   "-y", "-0.50",
                   "-z", "0.01",
                   "-Y", "0.0"],
        output="screen",
    )

    # controller_manager 由 gz_ros2_control 插件在 Gazebo 内提供
    # --controller-type 必须显式指定：gz_ros2_control 的 controller_manager
    # 解析 params_file 时可能丢失 type 参数（与独立 ros2_control_node 不同）
    param_file = os.path.join(pkg_share, "config",
                              "hardware_controller_manager.yaml")
    controller_args = [
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "60",
        "--param-file", param_file,
    ]

    joint_state = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-type",
                   "joint_state_broadcaster/JointStateBroadcaster",
                   *controller_args],
        output="screen",
    )
    diff_drive = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-type",
                   "diff_drive_controller/DiffDriveController",
                   *controller_args],
        output="screen",
    )
    imu = Node(
        package="controller_manager", executable="spawner",
        arguments=["imu_broadcaster",
                   "--controller-type",
                   "imu_sensor_broadcaster/IMUSensorBroadcaster",
                   *controller_args],
        output="screen",
    )
    arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller",
                   "--controller-type",
                   "joint_trajectory_controller/JointTrajectoryController",
                   *controller_args],
        output="screen",
    )
    # 两指都由它驱动（GripperActionController 是单关节的，驱不了 fr3_finger_joint2）
    gripper = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller",
                   "--controller-type",
                   "joint_trajectory_controller/JointTrajectoryController",
                   *controller_args],
        output="screen",
    )

    # ── 4. 传感器桥接：Ign topic → ROS topic ────────────────────
    #      格式: /topic@ROS_type[gz_type  (GZ→ROS)
    #            /topic@ROS_type]gz_type  (ROS→GZ)
    #            /topic@ROS_type@gz_type  (双向)
    #      重映射通过 Node 的 remappings 参数实现（非逗号语法！）

    # 传感器通过 <topic> 元素设定了短路径，直接用根级话题名。
    scan_topic = "/scan"
    image_topic = "/pi_camera/image"
    depth_topic = "/pi_camera/depth_image"
    camera_info_topic = "/pi_camera/camera_info"

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    lidar_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="lidar_bridge",
        arguments=[f"{scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        remappings=[(scan_topic, "/scan")],
        # 关键：Gazebo 传感器帧被加了模型名前缀 "turtlebot3/"，
        # 而 TF 树里的帧是未加前缀的 base_scan。必须强制覆盖，否则
        # AMCL/代价地图的 message filter 永远无法变换 /scan 而丢弃它。
        parameters=[{"override_frame_id": "base_scan"}],
        output="screen",
    )
    camera_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="camera_bridge",
        arguments=[f"{image_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(image_topic, "/camera/image_raw")],
        # 与 lidar 同理：Gazebo 给相机帧加 "turtlebot3/" 前缀，而 TF 树里是
        # 未加前缀的 camera_rgb_optical_frame，必须强制覆盖，否则 3D 定位的
        # TF 查询会失败。RGB/depth/camera_info 三者共用同一光学帧。
        parameters=[{"override_frame_id": "camera_rgb_optical_frame"}],
        output="screen",
    )
    depth_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="depth_bridge",
        arguments=[f"{depth_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(depth_topic, "/camera/depth/image_raw")],
        parameters=[{"override_frame_id": "camera_rgb_optical_frame"}],
        output="screen",
    )
    camera_info_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=[f"{camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        remappings=[(camera_info_topic, "/camera/camera_info")],
        parameters=[{"override_frame_id": "camera_rgb_optical_frame"}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz",
                                       "turtlebot3_manipulation.rviz")],
        parameters=[{"use_sim_time": True}],
        output="log",
        condition=IfCondition(rviz),
    )

    # ── 5. 顺序启动 ─────────────────────────────────────────────
    ld = LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="是否启动 RViz"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                               "/opt/ros/humble/lib/"),
        robot_state_publisher,
        gazebo,
        rviz_node,
        spawn_robot,
        clock_bridge,
        lidar_bridge,
        camera_bridge,
        depth_bridge,
        camera_info_bridge,
        # 控制器按顺序启动，避免并发 set_parameters 造成竞态条件
        # （FR3 demo 采用同样策略）
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state,
            on_exit=[diff_drive])),
        RegisterEventHandler(OnProcessExit(
            target_action=diff_drive,
            on_exit=[imu])),
        RegisterEventHandler(OnProcessExit(
            target_action=imu,
            on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(
            target_action=arm,
            on_exit=[gripper])),
    ])
    return ld
