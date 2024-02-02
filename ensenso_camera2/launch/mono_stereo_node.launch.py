from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ensenso_camera import ros2_launch as ensenso


def launch_setup(context, *args, **kwargs):
    stereo_settings = ensenso.fix_empty_string(LaunchConfiguration("stereo_settings").perform(context))
    stereo_file_camera_path = ensenso.fix_empty_string(LaunchConfiguration("stereo_path").perform(context))
    stereo_camera_frame = ensenso.fix_empty_string(LaunchConfiguration("camera_frame").perform(context))
    stereo_target_frame = ensenso.fix_empty_string(LaunchConfiguration("target_frame").perform(context))
    stereo_link_frame = ensenso.fix_empty_string(LaunchConfiguration("link_frame").perform(context))
    stereo_robot_frame = ensenso.fix_empty_string(LaunchConfiguration("robot_frame").perform(context))
    stereo_wrist_frame = ensenso.fix_empty_string(LaunchConfiguration("wrist_frame").perform(context))

    mono_settings = ensenso.fix_empty_string(LaunchConfiguration("mono_settings").perform(context))
    mono_file_camera_path = ensenso.fix_empty_string(LaunchConfiguration("mono_path").perform(context))

    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ensenso_camera",
                plugin="ensenso_camera::StereoCameraNode",
                name="stereo_node",
                namespace=LaunchConfiguration("stereo_ns"),
                parameters=[
                    {
                        "serial": LaunchConfiguration("stereo_serial"),
                        "settings": stereo_settings,
                        "file_camera_path": stereo_file_camera_path,
                        "fixed": LaunchConfiguration("stereo_fixed"),
                        "threads": LaunchConfiguration("stereo_threads"),
                        "camera_frame": stereo_camera_frame,
                        "target_frame": stereo_target_frame,
                        "link_frame": stereo_link_frame,
                        "robot_frame": stereo_robot_frame,
                        "wrist_frame": stereo_wrist_frame,
                        "tcp_port": LaunchConfiguration("stereo_tcp_port"),
                        "wait_for_camera": LaunchConfiguration("stereo_wait_for_camera"),
                    }
                ],
            ),
            ComposableNode(
                package="ensenso_camera",
                plugin="ensenso_camera::MonoCameraNode",
                name="mono_node",
                namespace=LaunchConfiguration("mono_ns"),
                parameters=[
                    {
                        "serial": LaunchConfiguration("mono_serial"),
                        "settings": mono_settings,
                        "file_camera_path": mono_file_camera_path,
                        "fixed": LaunchConfiguration("mono_fixed"),
                        "threads": LaunchConfiguration("mono_threads"),
                        # Link the mono to the stereo camera!
                        "link_frame": stereo_link_frame,
                        "tcp_port": LaunchConfiguration("mono_tcp_port"),
                        "wait_for_camera": LaunchConfiguration("mono_wait_for_camera"),
                        "capture_timeout": LaunchConfiguration("mono_capture_timeout"),
                    }
                ],
            ),
        ],
        output="screen",
    )

    return [container]


def generate_launch_description():
    stereo_namespace_launch_arg = DeclareLaunchArgument("stereo_ns", default_value="stereo")
    stereo_serial_launch_arg = DeclareLaunchArgument("stereo_serial", default_value="stereo_cam")
    stereo_settings_launch_arg = DeclareLaunchArgument("stereo_settings", default_value=ensenso.EMPTY_STRING)
    stereo_file_camera_path_launch_arg = DeclareLaunchArgument("stereo_path", default_value=ensenso.EMPTY_STRING)
    stereo_fixed_launch_arg = DeclareLaunchArgument("stereo_fixed", default_value="False")
    stereo_threads_launch_arg = DeclareLaunchArgument("stereo_threads", default_value="-1")
    stereo_camera_frame_launch_arg = DeclareLaunchArgument("camera_frame", default_value=ensenso.EMPTY_STRING)
    stereo_target_frame_launch_arg = DeclareLaunchArgument("target_frame", default_value=ensenso.EMPTY_STRING)
    stereo_link_frame_launch_arg = DeclareLaunchArgument("link_frame", default_value=ensenso.EMPTY_STRING)
    stereo_robot_frame_launch_arg = DeclareLaunchArgument("robot_frame", default_value=ensenso.EMPTY_STRING)
    stereo_wrist_frame_launch_arg = DeclareLaunchArgument("wrist_frame", default_value=ensenso.EMPTY_STRING)
    stereo_tcp_port_launch_arg = DeclareLaunchArgument("stereo_tcp_port", default_value="-1")
    stereo_wait_for_camera_launch_arg = DeclareLaunchArgument("stereo_wait_for_camera", default_value="False")

    mono_namespace_launch_arg = DeclareLaunchArgument("mono_ns", default_value="mono")
    mono_serial_launch_arg = DeclareLaunchArgument("mono_serial", default_value="mono_cam")
    mono_settings_launch_arg = DeclareLaunchArgument("mono_settings", default_value=ensenso.EMPTY_STRING)
    mono_file_camera_path_launch_arg = DeclareLaunchArgument("mono_path", default_value=ensenso.EMPTY_STRING)
    mono_fixed_launch_arg = DeclareLaunchArgument("mono_fixed", default_value="False")
    mono_threads_launch_arg = DeclareLaunchArgument("mono_threads", default_value="-1")
    mono_tcp_port_launch_arg = DeclareLaunchArgument("mono_tcp_port", default_value="-1")
    mono_wait_for_camera_launch_arg = DeclareLaunchArgument("mono_wait_for_camera", default_value="False")
    mono_capture_timeout_arg = DeclareLaunchArgument("mono_capture_timeout", default_value="1000")

    return LaunchDescription(
        [
            stereo_namespace_launch_arg,
            stereo_serial_launch_arg,
            stereo_settings_launch_arg,
            stereo_file_camera_path_launch_arg,
            stereo_fixed_launch_arg,
            stereo_threads_launch_arg,
            stereo_tcp_port_launch_arg,
            stereo_wait_for_camera_launch_arg,
            stereo_camera_frame_launch_arg,
            stereo_target_frame_launch_arg,
            stereo_link_frame_launch_arg,
            stereo_robot_frame_launch_arg,
            stereo_wrist_frame_launch_arg,
            mono_namespace_launch_arg,
            mono_serial_launch_arg,
            mono_settings_launch_arg,
            mono_file_camera_path_launch_arg,
            mono_fixed_launch_arg,
            mono_threads_launch_arg,
            mono_tcp_port_launch_arg,
            mono_wait_for_camera_launch_arg,
            mono_capture_timeout_arg,
            # Place Node/ComposableNodeContainer objects at the end, otherwise the
            # arguments from above are unknown to the objects.
            OpaqueFunction(function=launch_setup),
        ]
    )
