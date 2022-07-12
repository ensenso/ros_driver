import pytest

import ensenso_camera_test.ros2_testing_launch as launch
import ensenso_camera_test.ros2_testing_hz as hz

from sensor_msgs.msg import PointCloud2


params = hz.HzTestParameters(rate_in_hz=1.0, tolerance_in_hz=0.5, duration_in_s=20)

TestTexturePointCloud = hz.HzTest("test_texture_point_cloud", PointCloud2, "textured_point_cloud", params)


@pytest.mark.launch_test
def generate_test_description():
    return launch.get_launch_description(
        launch.EnsensoLaunchInclude(
            name="stereo_node.launch.py",
            args={
                "serial": "texture_cam",
                "file_camera_path": launch.get_data_path("stereo_camera/camera.zip"),
            },
        ),
        launch.EnsensoLaunchInclude(name="texture_point_cloud.launch.py"),
        launch.EnsensoScriptInclude(
            name="request_data",
            args={
                "rate": params.rate_in_hz,
                "raw_images": False,
                "rectified_images": False,
                "disparity_map": False,
                "normals": False,
            },
        ),
    )
