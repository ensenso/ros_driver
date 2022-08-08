import pytest

import ensenso_camera_test.ros2_testing_launch as launch

from ensenso_camera_test.hand_eye_calibration import TestHandEyeCalibration


@pytest.mark.launch_test
def generate_test_description():
    return launch.generate_test_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "hand_calib_cam",
                "file_camera_path": launch.get_data_path("hand_eye_calibration/camera.zip"),
                "camera_frame": "robot_wrist",
                "robot_frame": "robot_base",
            },
        )
    )
