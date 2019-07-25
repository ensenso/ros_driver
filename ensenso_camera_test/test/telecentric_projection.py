#!/usr/bin/env python
import rospy
import rostest

import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import TelecentricProjectionAction, TelecentricProjectionGoal, TelecentricProjectionResult
from ensenso_camera_msgs.msg import RequestDataAction, RequestDataGoal

from geometry_msgs.msg import Transform
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.point_cloud2 import PointCloud2

PATH = "../data/telecentric_projection"
IMAGE = PATH + "/image/reference_image.jpg"
TIMEOUT = 20
FRAME = "Workspace"


class TestTelecentricProjection(unittest.TestCase):
    def setUp(self):
        self.telecentric_projection_client = actionlib.SimpleActionClient("project_telecentric",
                                                                          TelecentricProjectionAction)
        self.request_data_client = actionlib.SimpleActionClient("request_data", RequestDataAction)
        for client in [self.telecentric_projection_client, self.request_data_client]:
            if not client.wait_for_server(rospy.Duration(TIMEOUT)):
                self.fail(msg="Request_data action servers timed out!")

        self.pc_subscriber = rospy.Subscriber("/projected_point_cloud", PointCloud2, self.callback)

        # Rotation, that is 90 degrees rotated to the original camera in the test
        self.trafo = Transform()
        self.trafo.rotation.x = 0
        self.trafo.rotation.y = -0.7071068
        self.trafo.rotation.z = 0
        self.trafo.rotation.w = 0.7071068
        self.trafo.translation.x = 0
        self.trafo.translation.y = 0
        self.trafo.translation.z = 0

        # Test the projected depth image
        self.request_point_cloud()
        self.send_goal_depth_image()
        self.generate_line_estimate()

        # Test the point cloud
        self.send_goal_point_cloud()
        self.get_projected_point_cloud()

        # Test the subscribed cloud
        self.send_goal_with_publishing_point_cloud()

    def request_point_cloud(self):
        request_data_goal = RequestDataGoal()
        request_data_goal.request_point_cloud = True
        self.request_data_client.send_goal(request_data_goal)
        self.request_data_client.wait_for_result()

        result = self.request_data_client.get_result()
        self.assertEqual(result.error.code, 0, msg="Requesting point cloud not successful")

    def send_goal_depth_image(self):
        # Setup the view pose to be 90 degrees rotated w.r.t the original camera
        goal = TelecentricProjectionGoal()
        goal.view_pose = self.trafo
        goal.frame = FRAME
        goal.request_depth_image = True
        goal.include_results_in_response = True
        self.telecentric_projection_client.send_goal(goal)

    def send_goal_point_cloud(self):
        goal = TelecentricProjectionGoal()
        goal.view_pose = self.trafo
        goal.frame = FRAME
        goal.include_results_in_response = True
        self.telecentric_projection_client.send_goal(goal)

    def generate_line_estimate(self):
        self.wait_for_server_succeed()

        result = self.telecentric_projection_client.get_result()
        image = result.projected_depth_map
        bridge = CvBridge()

        # Message to cv image
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

        # Convert it to a mono image via numpy
        mono8 = np.uint8(cv_image)
        cv.imwrite(PATH + "/image/mono8.jpg", mono8)

        # Grey image to binary image
        ret, binary_img = cv.threshold(mono8, 10, 255, cv.THRESH_BINARY)
        cv.imwrite(PATH + "/image/binary.jpg", binary_img)

        # Filter out non null pixels
        xyPoints = []
        height, width = binary_img.shape
        for i in range(0, height):
            for j in range(0, (width)):
                if binary_img[i, j] != 0:
                    point = (i, j)
                    xyPoints.append(point)

        # v is the normalized vector that is colinear to the found line estimate
        vx, vy, cx, cy = cv.fitLine(np.array(xyPoints), cv.DIST_L2, 0, 0.01, 0.01)

        self.line_parameters = [vx, vy, cx, cy]

    def get_projected_point_cloud(self):
        self.wait_for_server_succeed()
        result = self.telecentric_projection_client.get_result()
        self.projected_pc = result.projected_point_cloud

    def wait_for_server_succeed(self):
        if not self.telecentric_projection_client.wait_for_result(rospy.Duration(TIMEOUT)):
            self.fail(msg="Waiting for result times out.")
        if self.telecentric_projection_client.get_state() != GoalStatus.SUCCEEDED:
            self.fail(msg="Action did not succeed.")

    def callback(self, data):
        self.subscribed_cloud = data

    def send_goal_with_publishing_point_cloud(self):
        goal = TelecentricProjectionGoal()
        goal.view_pose = self.trafo
        goal.frame = FRAME
        goal.publish_results = True
        self.telecentric_projection_client.send_goal(goal)

    def test_found_line_paramters(self):
        self.assertAlmostEqual(self.line_parameters[0], 1.0, delta=0.05)
        self.assertAlmostEqual(self.line_parameters[1], 0.0, delta=0.05)

    def test_projected_point_cloud(self):
        cloud_points = list(pc2.read_points(self.projected_pc, skip_nans=True))
        self.assertTrue(len(cloud_points) != 0, msg=" The recieved point cloud is empty.")

    def test_subscribed_point_cloud(self):
        cloud_points = list(pc2.read_points(self.subscribed_cloud, skip_nans=True))
        self.assertTrue(len(cloud_points) != 0, msg=" The recieved point cloud is empty.")


if __name__ == "__main__":
    try:
        rospy.init_node("test_telecentric_projection")
        rostest.rosrun("ensenso_camera_test", "test_telecentric_projection", TestTelecentricProjection)
    except rospy.ROSInterruptException:
        pass
