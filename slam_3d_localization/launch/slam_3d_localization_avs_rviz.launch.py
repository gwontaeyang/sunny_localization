from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="slam_3d_localization",
            executable="slam_3d_localizer_node",
            name="slam_3d_localizer",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
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
    ])
