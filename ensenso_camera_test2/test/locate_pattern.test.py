import pytest

import ensenso_camera_test.ros2_testing_launch as launch

from ensenso_camera_test.locate_pattern import TestLocatePattern


@pytest.mark.launch_test
def generate_test_description():
    return launch.generate_test_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "pat_cam",
                "file_camera_path": launch.get_data_path("locate_pattern/camera.zip"),
            },
        )
    )
