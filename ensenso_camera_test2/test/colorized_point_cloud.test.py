import pytest
import ensenso_camera_test.ros2_testing_launch as launch

from ensenso_camera_test.colorized_point_cloud import TestColorizedPointCloud


@pytest.mark.launch_test
def generate_test_description():
    return launch.get_launch_description(
        launch.EnsensoLaunchInclude(
            name="mono_stereo_node.launch.py",
            args={
                "stereo_serial": "160606!",
                "stereo_path": launch.get_data_path("colorized_point_cloud/stereo.zip"),
                "stereo_ns": "stereo",
                "mono_serial": "color_mono",
                "mono_path": launch.get_data_path("colorized_point_cloud/mono.zip"),
                "mono_ns": "mono",
            },
        )
    )
