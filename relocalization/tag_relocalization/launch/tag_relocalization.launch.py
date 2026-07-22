#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("tag_relocalization")
    config_file = os.path.join(pkg_dir, "config", "tag_relocalization_params.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            Node(
                package="tag_relocalization",
                executable="tag_relocalization_node",
                name="tag_relocalization_node",
                output="screen",
                parameters=[
                    config_file,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
