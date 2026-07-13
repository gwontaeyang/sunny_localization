#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("amcl_guard")
    params_file = os.path.join(pkg_dir, "config", "amcl_guard_params.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
    )
    declare_output = DeclareLaunchArgument(
        "amcl_guard_output",
        default_value="log",
        description="Output mode for amcl_guard node",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_output,
            Node(
                package="amcl_guard",
                executable="amcl_guard_node",
                name="amcl_guard",
                output=LaunchConfiguration("amcl_guard_output"),
                parameters=[
                    params_file,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
