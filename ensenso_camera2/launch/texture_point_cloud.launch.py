from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    parameter_set_launch_arg = DeclareLaunchArgument("parameter_set", default_value="texture_point_cloud")

    image_stream_script = Node(
        package="ensenso_camera",
        executable="image_stream",
        name="image_stream",
        parameters=[
            {
                "parameter_set": LaunchConfiguration("parameter_set"),
                "rectified": True,
            }
        ],
    )

    texture_point_cloud_node = Node(
        package="ensenso_camera",
        executable="texture_point_cloud",
        name="texture_point_cloud",
    )

    return LaunchDescription(
        [
            parameter_set_launch_arg,
            image_stream_script,
            texture_point_cloud_node,
        ]
    )
