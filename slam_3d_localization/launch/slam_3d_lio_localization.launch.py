from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
                params_file,
                {
                    "map_path": LaunchConfiguration("map_path"),
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
