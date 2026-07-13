#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('state_estimation')
    
    # Path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'state_estimation_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        use_sim_time_arg,
        LifecycleNode(
            package='state_estimation',
            executable='state_estimation_node',
            name='state_estimation_node',
            namespace='',
            output='screen',
            parameters=[
                config_file,
                {'localizer_type': 'EKF'},
                {'node_name': 'state_estimation_node'},
                {"use_sim_time": use_sim_time}
            ],
            remappings=[],
        ),
    ])
