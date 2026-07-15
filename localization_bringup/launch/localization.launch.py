import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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


def generate_launch_description():
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
    launch_state_estimation_arg = DeclareLaunchArgument(
        "launch_state_estimation",
        default_value="true",
        description="Launch wheel/IMU state estimation.",
    )
    launch_amcl_arg = DeclareLaunchArgument(
        "launch_amcl",
        default_value="false",
        description="Launch AMCL map localization.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            use_sim_time_arg,
            robot_model_arg,
            type_arg,
            mode_arg,
            launch_state_estimation_arg,
            launch_amcl_arg,
            create_package_launch(
                "state_estimation",
                "state_estimation.launch.py",
                {"use_sim_time": use_sim_time},
                condition=IfCondition(LaunchConfiguration("launch_state_estimation")),
            ),
            create_package_launch(
                "amcl",
                "amcl.launch.py",
                {"use_sim_time": use_sim_time},
                condition=IfCondition(LaunchConfiguration("launch_amcl")),
            ),
        ]
    )
