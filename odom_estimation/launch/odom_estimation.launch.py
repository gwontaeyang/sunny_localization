#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('odom_estimation')
    
    # Path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'odom_estimation_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    load_initial_gyro_bias_arg = DeclareLaunchArgument(
        "load_initial_gyro_bias",
        default_value="false",
        description="Seed the EKF gyro z-bias from a calibration YAML",
    )
    gyro_bias_yaml_path_arg = DeclareLaunchArgument(
        "gyro_bias_yaml_path",
        default_value="~/.ros/imu_calibration.yaml",
        description="IMU calibration YAML path",
    )
    allow_initial_gyro_bias_fallback_arg = DeclareLaunchArgument(
        "allow_initial_gyro_bias_fallback",
        default_value="false",
        description="Use fallback YAML only when the active YAML does not exist",
    )
    gyro_bias_fallback_yaml_path_arg = DeclareLaunchArgument(
        "gyro_bias_fallback_yaml_path",
        default_value="",
        description="Validated startup fallback calibration YAML",
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    load_initial_gyro_bias = LaunchConfiguration("load_initial_gyro_bias")
    gyro_bias_yaml_path = LaunchConfiguration("gyro_bias_yaml_path")
    allow_initial_gyro_bias_fallback = LaunchConfiguration(
        "allow_initial_gyro_bias_fallback"
    )
    gyro_bias_fallback_yaml_path = LaunchConfiguration(
        "gyro_bias_fallback_yaml_path"
    )

    return LaunchDescription([
        use_sim_time_arg,
        load_initial_gyro_bias_arg,
        gyro_bias_yaml_path_arg,
        allow_initial_gyro_bias_fallback_arg,
        gyro_bias_fallback_yaml_path_arg,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link_tf",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "base_link",
            ],
            output="screen",
        ),
        LifecycleNode(
            package='odom_estimation',
            executable='odom_estimation_node',
            name='odom_estimation_node',
            namespace='',
            output='screen',
            parameters=[
                config_file,
                {'odom_estimator_type': 'EKF'},
                {'node_name': 'odom_estimation_node'},
                {"use_sim_time": use_sim_time},
                {
                    "load_initial_gyro_bias": ParameterValue(
                        load_initial_gyro_bias, value_type=bool
                    )
                },
                {
                    "gyro_bias_yaml_path": ParameterValue(
                        gyro_bias_yaml_path, value_type=str
                    )
                },
                {
                    "allow_initial_gyro_bias_fallback": ParameterValue(
                        allow_initial_gyro_bias_fallback, value_type=bool
                    )
                },
                {
                    "gyro_bias_fallback_yaml_path": ParameterValue(
                        gyro_bias_fallback_yaml_path, value_type=str
                    )
                },
            ],
            remappings=[],
        ),
    ])
