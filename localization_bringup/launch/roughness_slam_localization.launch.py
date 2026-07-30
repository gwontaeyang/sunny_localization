#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Get package directory (install directory)
    localization_bringup_dir = get_package_share_directory('localization_bringup')

    # Path to config files
    amcl_config_file = os.path.join(
        localization_bringup_dir,
        'config',
        'roughness_slam_amcl_params.yaml',
    )
    amcl_guard_config_file = os.path.join(
        get_package_share_directory('amcl_guard'), 'config', 'amcl_guard_params.yaml'
    )

    default_map_file = os.path.join(
        get_package_share_directory('amcl'),
        'map',
        'map_1763366890.yaml',
    )

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=amcl_config_file,
        description='Deprecated alias for amcl_params_file'
    )

    declare_amcl_params_file = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=LaunchConfiguration('params_file'),
        description='Full path to the AMCL parameters file to use'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_amcl_guard_output = DeclareLaunchArgument(
        'amcl_guard_output',
        default_value='log',
        description='Output mode for the amcl_guard node'
    )

    # AMCL scan guard node
    amcl_guard_node = Node(
        package='amcl_guard',
        executable='amcl_guard_node',
        name='amcl_guard',
        output=LaunchConfiguration('amcl_guard_output'),
        parameters=[
            amcl_guard_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # AMCL Node
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('amcl_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/map', '/map_amcl'),
            ('/initialpose', '/initialpose'),
            ('/amcl_pose', '/amcl_pose'),
            ('/particlecloud', '/particlecloud')
        ]
    )

    # Map Server Node
    map_server_node = LifecycleNode(
        package='nav_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Lifecycle Manager Node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        declare_params_file,
        declare_amcl_params_file,
        declare_autostart,
        declare_amcl_guard_output,
        amcl_guard_node,
        amcl_node,
        map_server_node,
        lifecycle_manager_node,
    ])
