import pytest

import ensenso_camera_test.ros2_testing_launch as launch
import ensenso_camera_test.ros2_testing_hz as hz

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

params = hz.HzTestParameters(rate_in_hz=1.0, tolerance_in_hz=0.5, duration_in_s=15)

TestRawLeftImage = hz.HzTest("test_structured_light_raw_left_image", Image, "raw/left/image", params)
TestRectifiedLeftImage = hz.HzTest("test_structured_light_rectified_left_image", Image, "rectified/left/image", params)
TestPointCloud = hz.HzTest("test_structured_light_point_cloud", PointCloud2, "point_cloud", params)
TestDepthImage = hz.HzTest("test_structured_light_depth_image", Image, "depth/image", params)


@pytest.mark.launch_test
def generate_test_description():
    return launch.get_launch_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "req_cam_s",
                "file_camera_path": launch.get_data_path("structured_light_camera/camera.zip"),
            },
        ),
        launch.EnsensoScriptInclude(
            name="request_data",
            args={"rate": params.rate_in_hz},
        ),
    )
