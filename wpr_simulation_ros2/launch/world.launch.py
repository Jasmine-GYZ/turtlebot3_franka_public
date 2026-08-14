"""Launch a WPR world in Gazebo Fortress."""

from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_VALID_WORLDS = {"example", "official"}
_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}


def _boolean_argument(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in _TRUE_VALUES:
        return True
    if value in _FALSE_VALUES:
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be a boolean value, got: {value!r}"
    )


def _launch_setup(context):
    package_share = Path(get_package_share_directory("wpr_simulation_ros2"))
    world_type = LaunchConfiguration("world_type").perform(context)
    if world_type not in _VALID_WORLDS:
        valid_values = ", ".join(sorted(_VALID_WORLDS))
        raise RuntimeError(
            f"Unknown world_type {world_type!r}; expected one of: {valid_values}"
        )

    world_path = package_share / "worlds" / f"{world_type}.world"
    if not world_path.is_file():
        raise RuntimeError(f"World file does not exist: {world_path}")

    gz_arguments = []
    if not _boolean_argument(context, "gui"):
        gz_arguments.append("-s")
    if not _boolean_argument(context, "paused"):
        gz_arguments.append("-r")
    gz_arguments.extend(["-v", "4" if _boolean_argument(context, "verbose") else "2"])
    gz_arguments.append(str(world_path))

    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": shlex.join(gz_arguments),
            "gz_version": "6",
            "on_exit_shutdown": "true",
        }.items(),
    )

    actions = [gazebo]
    if _boolean_argument(context, "bridge_clock"):
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_type",
                default_value="example",
                description="World to load: example or official",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical interface",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start with simulation physics paused",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Enable Gazebo debug-level console output",
            ),
            DeclareLaunchArgument(
                "bridge_clock",
                default_value="true",
                description="Bridge Gazebo simulation time to the ROS 2 /clock topic",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
