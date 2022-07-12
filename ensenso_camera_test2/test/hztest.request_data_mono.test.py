import pytest

import ensenso_camera_test.ros2_testing_launch as launch
import ensenso_camera_test.ros2_testing_hz as ros2_testing_launch

from sensor_msgs.msg import Image

params = ros2_testing_launch.HzTestParameters(rate_in_hz=1.0, tolerance_in_hz=0.5, duration_in_s=15)

TestRawImage = ros2_testing_launch.HzTest("test_mono_raw_image", Image, "raw/image", params)
TestRectfiedImage = ros2_testing_launch.HzTest("test_mono_rectified_image", Image, "rectified/image", params)


@pytest.mark.launch_test
def generate_test_description():
    return launch.get_launch_description(
        launch.EnsensoLaunchInclude(
            name="mono_node.launch.py",
            args={
                "serial": "req_cam_mono",
                "file_camera_path": launch.get_data_path("mono_camera/camera.zip"),
            },
        ),
        launch.EnsensoScriptInclude(
            name="request_data_mono",
            args={"rate": params.rate_in_hz},
        ),
    )
