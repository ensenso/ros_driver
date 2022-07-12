#!/usr/bin/env python
import unittest

import ctypes
import struct

import ensenso_camera.ros2 as ros2py

import ensenso_camera_test.ros2_testing as ros2py_testing

pc2 = ros2py_testing.import_point_cloud2()

TexturedPointCloud = ros2py.import_action("ensenso_camera_msgs", "TexturedPointCloud")
RequestDataMono = ros2py.import_action("ensenso_camera_msgs", "RequestDataMono")
RequestData = ros2py.import_action("ensenso_camera_msgs", "RequestData")

STEREO_NAMESPACE = "stereo"
MONO_NAMESPACE = "mono"
MONO_SERIAL = "color_mono"


class TestColorizedPointCloud(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_colorized_point_cloud")
        self.request_stereo_client = ros2py.create_action_client(
            self.node, STEREO_NAMESPACE + "/request_data", RequestData
        )
        self.request_mono_client = ros2py.create_action_client(
            self.node, MONO_NAMESPACE + "/request_data", RequestDataMono
        )
        self.colorize_point_cloud_client = ros2py.create_action_client(
            self.node, STEREO_NAMESPACE + "/texture_point_cloud", TexturedPointCloud
        )

        clients = [self.request_stereo_client, self.request_mono_client, self.colorize_point_cloud_client]
        success = ros2py.wait_for_servers(self.node, clients, timeout_sec=ros2py_testing.TEST_TIMEOUT, exit=False)
        self.assertTrue(success, msg="Timeout reached for servers.")
        self.colored_point_cloud = None

    def test_colorize_point_cloud(self):
        self.send_data_goals()
        self.aquire_data()
        self.recieved_point_cloud()
        self.check_color()

    def send_data_goals(self):
        goal_stereo = RequestData.Goal()
        goal_stereo.request_point_cloud = True
        goal_mono = RequestDataMono.Goal()
        goal_mono.request_rectified_images = True

        goals = [goal_stereo, goal_mono]
        clients = [self.request_stereo_client, self.request_mono_client]
        stereo_response, mono_response = ros2py.send_action_goals(self.node, clients, goals)

        self.assertFalse(stereo_response.timeout(), msg="Timeout reached for stereo results.")
        self.assertFalse(mono_response.timeout(), msg="Timeout reached for mono results.")

        self.assertTrue(stereo_response.successful(), msg="Request stereo data action has not been successful.")
        self.assertTrue(mono_response.successful(), msg="Request mono data action has not been successful.")

    def aquire_data(self):
        goal_texture = TexturedPointCloud.Goal()
        goal_texture.use_opengl = False
        goal_texture.include_results_in_response = True
        goal_texture.mono_serial = MONO_SERIAL
        goal_texture.far_plane = 4000.0
        goal_texture.near_plane = 100.0

        response = ros2py.send_action_goal(self.node, self.colorize_point_cloud_client, goal_texture)

        self.assertFalse(response.timeout(), msg="Timeout reached for results.")
        self.assertTrue(response.successful(), msg="Request textured point cloud action has not been successful.")

        self.colored_point_cloud = response.get_result()

    def recieved_point_cloud(self):
        result = self.colored_point_cloud
        self.assertEqual(result.error.message, "", msg="Got an error with the result")
        self.cloud_points = list(pc2.read_points(result.point_cloud, skip_nans=True))
        self.assertTrue(
            len(self.cloud_points) != 0,
            msg="The recieved point cloud is empty. Recieved {} points".format(len(self.cloud_points)),
        )

    def check_color(self):
        for p in self.cloud_points:
            # p has 4 points. The last one holds the color information.
            # (http://answers.ros.org/question/208834/read-colours-from-a-pointcloud2-python/)

            test = p[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack(">f", test)
            i = struct.unpack(">l", s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = pack & 0x000000FF

            self.assertTrue(r != 0.0, msg="r-value cannot be zero")
            self.assertTrue(g != 0.0, msg="g-value cannot be zero")
            self.assertTrue(b != 0.0, msg="b-value cannot be zero")


def main():
    ros2py_testing.run_ros1_test("test_colorized_point_cloud", TestColorizedPointCloud)


if __name__ == "__main__":
    main()
