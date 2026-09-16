import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_nodes(context):
    package_share = FindPackageShare("slam_3d_localization")
    params_file = PathJoinSubstitution([
        package_share,
        "config",
        "slam_3d_localization_avs.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        package_share,
        "rviz",
        "slam_3d_localization_avs.rviz",
    ])

    path = Path(__file__).with_name("lio_profile.py")
    spec = importlib.util.spec_from_file_location("slam_3d_registration_profile", path)
    profile = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(profile)
    runtime = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        **profile.registration_method_override(
            LaunchConfiguration("registration_method").perform(context)
        ),
    }
    return [
        Node(
            package="slam_3d_localization",
            executable="slam_3d_localizer_node",
            name="slam_3d_localizer",
            output="screen",
            parameters=[
                params_file,
                runtime,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="slam_3d_localization_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "registration_method", default_value="",
            description="Optional matcher override; empty keeps the AVS params file.",
        ),
        OpaqueFunction(function=create_nodes),
    ])
