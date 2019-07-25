#!/usr/bin/env python
import rospy
import rostest

import unittest

import actionlib

from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import TexturedPointCloudAction, TexturedPointCloudGoal
from ensenso_camera_msgs.msg import RequestDataMonoAction, RequestDataMonoGoal
from ensenso_camera_msgs.msg import RequestDataAction, RequestDataGoal

import ctypes
import struct

import sensor_msgs.point_cloud2 as pc2

WAIT_TIMEOUT = 20

STEREO_NAMESPACE = "stereo"
MONO_NAMESPACE = "mono"
MONO_SERIAL = "4002895081"


class TestColorizedPointCloud(unittest.TestCase):
    def setUp(self):
        self.request_stereo = actionlib.SimpleActionClient(STEREO_NAMESPACE + "/request_data", RequestDataAction)
        self.request_mono = actionlib.SimpleActionClient(MONO_NAMESPACE + "/request_data", RequestDataMonoAction)
        self.colorize_point_cloud = actionlib.SimpleActionClient(STEREO_NAMESPACE + "/texture_point_cloud",
                                                                 TexturedPointCloudAction)
        for client in [self.request_mono, self.request_stereo,
                       self.colorize_point_cloud]:
            if not client.wait_for_server(rospy.Duration(WAIT_TIMEOUT)):
                self.fail(msg="Timeout reached for servers.")

    def test_colorize_point_cloud(self):
        self.send_data_goals()
        self.aquire_data()
        self.recieved_point_cloud()
        self.check_color()

    def send_data_goals(self):
        goal_mono = RequestDataMonoGoal()
        goal_mono.request_rectified_images = True
        goal = RequestDataGoal()
        goal.request_point_cloud = True
        self.request_mono.send_goal(goal_mono)
        self.request_stereo.send_goal(goal)

        for client in [self.request_mono, self.request_stereo]:
            if not client.wait_for_result(rospy.Duration(WAIT_TIMEOUT)):
                self.fail(msg="Timeout reached for results.")
            self.assertTrue(client.get_state() == GoalStatus.SUCCEEDED, msg="Request action has not been successful.")

    def aquire_data(self):
        goal_texture = TexturedPointCloudGoal()
        goal_texture.use_openGL = True
        goal_texture.include_results_in_response = True
        goal_texture.mono_serial = MONO_SERIAL
        goal_texture.far_plane = 4000.0
        goal_texture.near_plane = 100.0
        self.colorize_point_cloud.send_goal(goal_texture)
        if not self.colorize_point_cloud.wait_for_result(rospy.Duration(WAIT_TIMEOUT)):
            self.fail(msg="Timeout reached for results.")

    def recieved_point_cloud(self):
        result = self.colorize_point_cloud.get_result()
        self.assertEqual(result.error.message, "", msg="Got an error with the result")
        self.cloud_points = list(pc2.read_points(result.point_cloud, skip_nans=True))
        self.assertTrue(len(self.cloud_points) != 0, msg=" The recieved point cloud is empty.")

    def check_color(self):
        for p in self.cloud_points:
            # p has 4 points. The last one holds the color information. (http://answers.ros.org/question/208834/read-colours-from-a-pointcloud2-python/)
            test = p[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f', test)
            i = struct.unpack('>l', s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            self.assertTrue(r != 0., msg="r-value cannot be zero")
            self.assertTrue(g != 0., msg="g-value cannot be zero")
            self.assertTrue(b != 0., msg="b-value cannot be zero")


if __name__ == "__main__":
    try:
        rospy.init_node("test_colorized_point_cloud")
        rostest.rosrun("ensenso_camera_test", "test_colorized_point_cloud", TestColorizedPointCloud)
    except rospy.ROSInterruptException:
        pass
