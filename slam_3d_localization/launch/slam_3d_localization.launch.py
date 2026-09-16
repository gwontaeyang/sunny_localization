import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def create_nodes(context):
    path = Path(__file__).with_name("lio_profile.py")
    spec = importlib.util.spec_from_file_location("slam_3d_registration_profile", path)
    profile = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(profile)
    runtime = {
        "map_path": LaunchConfiguration("map_path"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        **profile.registration_method_override(
            LaunchConfiguration("registration_method").perform(context)
        ),
    }
    return [Node(
        package="slam_3d_localization",
        executable="slam_3d_localizer_node",
        name="slam_3d_localizer",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), runtime],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"),
                "analysis_outputs",
                "slam_3d",
                "30_floor_person_filtered_nav_grid",
                "full_pcd_maps_only",
                "gangnam_floor_person_filtered_nav_classified_rgb.pcd",
            ]),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("slam_3d_localization"),
                "config",
                "slam_3d_localization.yaml",
            ]),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "registration_method", default_value="",
            description="Optional matcher override; empty keeps params_file/defaults.",
        ),
        OpaqueFunction(function=create_nodes),
    ])
