import pytest

import ensenso_camera_test.ros2_testing_launch as launch

from ensenso_camera_test.locate_multiple_patterns import TestLocateMultiplePatterns


@pytest.mark.launch_test
def generate_test_description():
    return launch.get_launch_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "multi_pat_cam",
                "file_camera_path": launch.get_data_path("locate_multiple_patterns/camera.zip"),
            },
        )
    )
