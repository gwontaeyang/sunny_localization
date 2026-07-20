import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def create_package_launch(package_name, launch_file, launch_args=None, condition=None):
    kwargs = {
        "launch_description_source": PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", launch_file)
        ),
        "launch_arguments": (launch_args or {}).items(),
    }
    if condition is not None:
        kwargs["condition"] = condition
    return IncludeLaunchDescription(**kwargs)


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), ".ros", "db", "maps", LaunchConfiguration("map")]
    )
    system_type = LaunchConfiguration("type").perform(context).strip().lower() or "apart"
    mode = LaunchConfiguration("mode").perform(context).strip().upper() or "NAV"

    actions = [
        create_package_launch(
            "odom_estimation",
            "odom_estimation.launch.py",
            {"use_sim_time": use_sim_time},
        ),
    ]

    if system_type == "apart" and mode == "NAV":
        actions.append(
            create_package_launch(
                "amcl",
                "amcl.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "map": map_yaml_file,
                },
            )
        )

    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory("localization_bringup")
    config_file = os.path.join(pkg_dir, "config", "localization_bringup.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="rover",
        description="Robot model name.",
    )
    type_arg = DeclareLaunchArgument("type", default_value="apart")
    mode_arg = DeclareLaunchArgument("mode", default_value="NAV")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="blank_map.yaml",
        description="Map filename to load from $HOME/.ros/db/maps for AMCL.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization_bringup_node = Node(
        package="localization_bringup",
        executable="localization_bringup_node",
        name="localization_bringup_node",
        parameters=[
            config_file,
            {
                "config_file_path": config_file,
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            robot_model_arg,
            type_arg,
            mode_arg,
            map_arg,
            localization_bringup_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
