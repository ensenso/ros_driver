import pytest

import ensenso_camera_test.ros2_testing_launch as launch

from ensenso_camera_test.scale_disparity_map import TestScaleDisparityMap

from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    return launch.generate_test_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "scale_dmap_cam",
                "file_camera_path": launch.get_data_path("colorized_point_cloud/stereo.zip"),
                "link_frame": "Workspace",
            },
        ),
        Node(
            package="depth_image_proc",
            executable="point_cloud_xyz_node",
            name="dip_point_cloud",
            remappings=[
                ("/camera_info", "/depth/camera_info"),
                ("/image_rect", "/depth/image"),
                ("/points", "/point_cloud_dip"),
            ],
        ),
    )
