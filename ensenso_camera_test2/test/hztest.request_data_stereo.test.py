import pytest

import ensenso_camera_test.ros2_testing_launch as launch
import ensenso_camera_test.ros2_testing_hz as hz

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

params = hz.HzTestParameters(rate_in_hz=1.0, tolerance_in_hz=0.5, duration_in_s=15)

TestRawLeftImage = hz.HzTest("test_stereo_raw_left_image", Image, "raw/left/image", params)
TestRawRightImage = hz.HzTest("test_stereo_raw_right_image", Image, "raw/right/image", params)
TestRectifiedLeftImage = hz.HzTest("test_stereo_rectified_left_image", Image, "rectified/left/image", params)
TestRectifiedRightImage = hz.HzTest("test_stereo_rectified_right_image", Image, "rectified/right/image", params)
TestDisparityMap = hz.HzTest("test_stereo_disparity_map", Image, "disparity_map", params)
TestPointCloud = hz.HzTest("test_stereo_point_cloud", PointCloud2, "point_cloud", params)
TestDepthImage = hz.HzTest("test_stereo_depth_image", Image, "depth/image", params)


@pytest.mark.launch_test
def generate_test_description():
    return launch.get_launch_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "req_cam",
                "file_camera_path": launch.get_data_path("stereo_camera/camera.zip"),
            },
        ),
        launch.EnsensoScriptInclude(
            name="request_data",
            args={"rate": params.rate_in_hz},
        ),
    )
