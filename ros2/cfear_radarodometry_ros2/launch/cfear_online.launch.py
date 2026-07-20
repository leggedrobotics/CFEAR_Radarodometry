"""Launch the online CFEAR radar odometry node with the CFEAR-3 Boreas preset.

Override topics or use_sim_time from the command line, e.g.:
    ros2 launch cfear_radarodometry_ros2 cfear_online.launch.py \
        fft_topic:=/radar_data/fft odom_topic:=/odometry
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    """Build the node, overriding fft_topic/odom_topic only when explicitly passed.

    An unconditional parameter dict would clobber whatever params_file sets for
    these keys (e.g. cfear3_b2w_ras3.yaml) with this launch file's own
    Boreas-flavoured defaults, since later entries in `parameters` win.
    """
    params = [LaunchConfiguration("params_file").perform(context)]
    # Pass the LaunchConfiguration itself (not .perform()'d to a plain str) so
    # launch_ros infers the bool type; use_sim_time is declared bool by rclcpp
    # and a literal str value here raises "invalid type" at node startup.
    overrides = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    fft_topic = LaunchConfiguration("fft_topic").perform(context)
    if fft_topic:
        overrides["radar.fft_topic"] = fft_topic
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    if odom_topic:
        overrides["out.odom_topic"] = odom_topic
    fft_reliability = LaunchConfiguration("fft_reliability").perform(context)
    if fft_reliability:
        overrides["radar.fft_reliability"] = fft_reliability
    params.append(overrides)

    return [
        Node(
            package="cfear_radarodometry_ros2",
            executable="cfear_odometry_node",
            name="cfear_odometry_node",
            output="screen",
            parameters=params,
        )
    ]


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("cfear_radarodometry_ros2"),
        "config",
        "cfear3_boreas.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("fft_topic", default_value="",
                               description="Overrides params_file's radar.fft_topic when non-empty."),
        DeclareLaunchArgument("odom_topic", default_value="",
                               description="Overrides params_file's out.odom_topic when non-empty."),
        DeclareLaunchArgument("fft_reliability", default_value="",
                               description="Overrides params_file's radar.fft_reliability when non-empty "
                               "(e.g. 'best_effort' to test against dataset_replay_node, which publishes "
                               "best_effort; the real navtech driver is reliable)."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=params),
        OpaqueFunction(function=_launch_setup),
    ])
