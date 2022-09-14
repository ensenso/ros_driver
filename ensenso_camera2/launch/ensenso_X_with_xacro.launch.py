import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ensenso_camera import ros2_launch as ensenso


def launch_setup(context, *args, **kwargs):
    serial = ensenso.fix_empty_string(LaunchConfiguration("serial").perform(context))
    settings = ensenso.fix_empty_string(LaunchConfiguration("settings").perform(context))
    file_camera_path = ensenso.fix_empty_string(LaunchConfiguration("file_camera_path").perform(context))
    camera_frame = ensenso.fix_empty_string(LaunchConfiguration("camera_frame").perform(context))
    target_frame = ensenso.fix_empty_string(LaunchConfiguration("target_frame").perform(context))
    link_frame = ensenso.fix_empty_string(LaunchConfiguration("link_frame").perform(context))
    robot_frame = ensenso.fix_empty_string(LaunchConfiguration("robot_frame").perform(context))
    wrist_frame = ensenso.fix_empty_string(LaunchConfiguration("wrist_frame").perform(context))

    frame_id = f"optical_frame_{serial}"
    xacro_reference_frame = LaunchConfiguration("xacro_reference_frame").perform(context)
    xacro_model_type = LaunchConfiguration("type").perform(context)
    xacro_model_path = os.path.join(
        get_package_share_directory("ensenso_description"), f"ensenso_{xacro_model_type}_Series.xacro"
    )
    xaxro_collision_margin = LaunchConfiguration("collision_margin").perform(context)

    transforms_dict = {
        "X200": ["0.1", "0.036", "0.033", "0.7071068", "0.0", "0.0", "0.7071068"],
        "X400": ["0.2", "0.036", "0.033", "0.7071068", "0.0", "0.0", "0.7071068"],
    }
    static_transform_publisher_args = transforms_dict[xacro_model_type.strip("_xFA")] + [
        frame_id,
        xacro_reference_frame,
    ]
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_model_broadcaster",
        arguments=static_transform_publisher_args,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="camera_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": xacro.process_file(
                    xacro_model_path,
                    mappings={
                        "camera": serial,
                        "corresponding_frame_name": xacro_reference_frame,
                        "margin": xaxro_collision_margin,
                    },
                ).toxml()
            }
        ],
        remappings=[("robot_description", "ensenso_description")],
    )

    stereo_launch_file = ensenso.LaunchFileInclude(
        name="stereo_node.launch.py",
        args={
            "serial": serial,
            "settings": settings,
            "file_camera_path": file_camera_path,
            "fixed": LaunchConfiguration("fixed"),
            "threads": LaunchConfiguration("threads"),
            "camera_frame": camera_frame,
            "target_frame": target_frame,
            "link_frame": link_frame,
            "robot_frame": robot_frame,
            "wrist_frame": wrist_frame,
            "tcp_port": LaunchConfiguration("tcp_port"),
            "wait_for_camera": LaunchConfiguration("wait_for_camera"),
        },
    ).get_launch_description()

    return [
        static_transform_publisher_node,
        robot_state_publisher_node,
        stereo_launch_file,
    ]


def generate_launch_description():
    # Camera parameters
    serial_launch_arg = DeclareLaunchArgument("serial", default_value=ensenso.EMPTY_STRING)
    settings_launch_arg = DeclareLaunchArgument("settings", default_value=ensenso.EMPTY_STRING)
    file_camera_path_launch_arg = DeclareLaunchArgument("file_camera_path", default_value=ensenso.EMPTY_STRING)
    fixed_launch_arg = DeclareLaunchArgument("fixed", default_value="False")
    threads_launch_arg = DeclareLaunchArgument("threads", default_value="-1")
    camera_frame_launch_arg = DeclareLaunchArgument("camera_frame", default_value=ensenso.EMPTY_STRING)
    target_frame_launch_arg = DeclareLaunchArgument("target_frame", default_value=ensenso.EMPTY_STRING)
    link_frame_launch_arg = DeclareLaunchArgument("link_frame", default_value=ensenso.EMPTY_STRING)
    robot_frame_launch_arg = DeclareLaunchArgument("robot_frame", default_value=ensenso.EMPTY_STRING)
    wrist_frame_launch_arg = DeclareLaunchArgument("wrist_frame", default_value=ensenso.EMPTY_STRING)
    tcp_port_launch_arg = DeclareLaunchArgument("tcp_port", default_value="-1")
    wait_for_camera_launch_arg = DeclareLaunchArgument("wait_for_camera", default_value="False")

    # Xacro parameters
    xacro_reference_frame_launch_arg = DeclareLaunchArgument("xacro_reference_frame", default_value="xacro_ref")
    collision_margin_launch_arg = DeclareLaunchArgument("collision_margin", default_value="0.03")
    type_launch_arg = DeclareLaunchArgument("type", default_value="X200")

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
            robot_frame_launch_arg,
            wrist_frame_launch_arg,
            tcp_port_launch_arg,
            wait_for_camera_launch_arg,
            xacro_reference_frame_launch_arg,
            collision_margin_launch_arg,
            type_launch_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
