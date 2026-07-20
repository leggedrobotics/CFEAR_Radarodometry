"""Launch the online CFEAR radar odometry node with the CFEAR-3 Boreas preset.

Override topics or use_sim_time from the command line, e.g.:
    ros2 launch cfear_radarodometry_ros2 cfear_online.launch.py \
        fft_topic:=/radar_data/fft odom_topic:=/odometry
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("cfear_radarodometry_ros2"),
        "config",
        "cfear3_boreas.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("fft_topic", default_value="/radar_data/fft"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=params),
        Node(
            package="cfear_radarodometry_ros2",
            executable="cfear_odometry_node",
            name="cfear_odometry_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "radar.fft_topic": LaunchConfiguration("fft_topic"),
                    "out.odom_topic": LaunchConfiguration("odom_topic"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
    ])
