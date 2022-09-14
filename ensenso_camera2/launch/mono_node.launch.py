from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ensenso_camera import ros2_launch as ensenso


def launch_setup(context, *args, **kwargs):
    settings = ensenso.fix_empty_string(LaunchConfiguration("settings").perform(context))
    file_camera_path = ensenso.fix_empty_string(LaunchConfiguration("file_camera_path").perform(context))
    camera_frame = ensenso.fix_empty_string(LaunchConfiguration("camera_frame").perform(context))
    target_frame = ensenso.fix_empty_string(LaunchConfiguration("target_frame").perform(context))
    link_frame = ensenso.fix_empty_string(LaunchConfiguration("link_frame").perform(context))

    node = Node(
        package="ensenso_camera",
        executable="ensenso_camera_mono_node",
        name="ensenso_camera_mono_node",
        parameters=[
            {
                "serial": LaunchConfiguration("serial"),
                "settings": settings,
                "file_camera_path": file_camera_path,
                "fixed": LaunchConfiguration("fixed"),
                "threads": LaunchConfiguration("threads"),
                "camera_frame": camera_frame,
                "target_frame": target_frame,
                "link_frame": link_frame,
                "tcp_port": LaunchConfiguration("tcp_port"),
                "wait_for_camera": LaunchConfiguration("wait_for_camera"),
            }
        ],
    )

    return [node]


def generate_launch_description():
    serial_launch_arg = DeclareLaunchArgument("serial", default_value="mono_cam")
    settings_launch_arg = DeclareLaunchArgument("settings", default_value=ensenso.EMPTY_STRING)
    file_camera_path_launch_arg = DeclareLaunchArgument("file_camera_path", default_value=ensenso.EMPTY_STRING)
    fixed_launch_arg = DeclareLaunchArgument("fixed", default_value="False")
    threads_launch_arg = DeclareLaunchArgument("threads", default_value="-1")
    camera_frame_launch_arg = DeclareLaunchArgument("camera_frame", default_value=ensenso.EMPTY_STRING)
    target_frame_launch_arg = DeclareLaunchArgument("target_frame", default_value=ensenso.EMPTY_STRING)
    link_frame_launch_arg = DeclareLaunchArgument("link_frame", default_value=ensenso.EMPTY_STRING)
    tcp_port_launch_arg = DeclareLaunchArgument("tcp_port", default_value="-1")
    wait_for_camera_launch_arg = DeclareLaunchArgument("wait_for_camera", default_value="False")

    return LaunchDescription(
        [
            serial_launch_arg,
            settings_launch_arg,
            file_camera_path_launch_arg,
            fixed_launch_arg,
            threads_launch_arg,
            camera_frame_launch_arg,
            target_frame_launch_arg,
            link_frame_launch_arg,
            tcp_port_launch_arg,
            wait_for_camera_launch_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
