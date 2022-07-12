import pytest

import ensenso_camera_test.ros2_testing_launch as launch

from ensenso_camera_test.workspace_calibration import TestWorkspaceCalibration


@pytest.mark.launch_test
def generate_test_description():
    return launch.get_launch_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "work_calib_cam",
                "file_camera_path": launch.get_data_path("stereo_camera/camera.zip"),
            },
        )
    )
