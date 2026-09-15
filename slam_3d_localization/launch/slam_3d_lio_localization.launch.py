import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_pipeline_parameters():
    path = Path(__file__).with_name("lio_profile.py")
    spec = importlib.util.spec_from_file_location("slam_3d_lio_profile", path)
    profile = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(profile)
    return profile.get_lio_pipeline_parameters()


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pipeline = load_pipeline_parameters()

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
                "slam_3d_lio_gangnam.yaml",
            ]),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="slam_3d_localization",
            executable="slam_3d_lio_odometry_node",
            name="slam_3d_lio_odometry",
            output="screen",
            parameters=[
                pipeline["slam_3d_lio_odometry"]["ros__parameters"],
                params_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="slam_3d_localization",
            executable="slam_3d_localizer_node",
            name="slam_3d_localizer",
            output="screen",
            parameters=[
                pipeline["slam_3d_localizer"]["ros__parameters"],
                params_file,
                {
                    "map_path": LaunchConfiguration("map_path"),
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
