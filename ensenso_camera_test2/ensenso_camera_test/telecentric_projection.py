#!/usr/bin/env python
import cv2 as cv
import numpy as np
import os
import unittest

import ensenso_camera.ros2 as ros2py

import ensenso_camera_test.ros2_testing as ros2py_testing

from geometry_msgs.msg import Transform
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2

pc2 = ros2py_testing.import_point_cloud2()

TelecentricProjection = ros2py.import_action("ensenso_camera_msgs", "TelecentricProjection")
RequestData = ros2py.import_action("ensenso_camera_msgs", "RequestData")

IMAGE_PATH = ros2py_testing.get_test_data_path("telecentric_projection/")
IMAGE_PATH_MONO8 = IMAGE_PATH + "mono8.jpg"
IMAGE_PATH_BINARY = IMAGE_PATH + "binary.jpg"

FRAME = "Workspace"


class TestTelecentricProjection(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_telecentric_projection")
        self.telecentric_projection_client = ros2py.create_action_client(
            self.node, "project_telecentric", TelecentricProjection
        )
        self.request_data_client = ros2py.create_action_client(self.node, "request_data", RequestData)

        clients = [self.telecentric_projection_client, self.request_data_client]
        success = ros2py.wait_for_servers(self.node, clients, timeout_sec=ros2py_testing.TEST_TIMEOUT, exit=False)
        self.assertTrue(success, msg="Timeout reached for servers.")

        self.pc_subscriber = ros2py.create_subscription(self.node, PointCloud2, "/projected_point_cloud", self.callback)
        self.got_subscribed_cloud = False

        # Rotation, that is 90 degrees rotated to the original camera in the test
        self.trafo = Transform()
        self.trafo.rotation.x = 0.0
        self.trafo.rotation.y = -0.7071068
        self.trafo.rotation.z = 0.0
        self.trafo.rotation.w = 0.7071068
        self.trafo.translation.x = 0.0
        self.trafo.translation.y = 0.0
        self.trafo.translation.z = 0.0

        self.timeout = ros2py.Duration(ros2py_testing.TEST_TIMEOUT)

        # Prepare the projected depth image
        self.request_point_cloud()
        self.retrieve_projected_depth_map()
        self.generate_line_estimate()  # --> gets/needs projected_depth_map

        # Prepare the projected point cloud
        self.retrieve_projected_point_cloud()

        # Test the subscribed cloud
        self.send_goal_with_publishing_point_cloud()

    def tearDown(self):
        if os.path.isfile(IMAGE_PATH_MONO8):
            os.remove(IMAGE_PATH_MONO8)
        if os.path.isfile(IMAGE_PATH_BINARY):
            os.remove(IMAGE_PATH_BINARY)
        # Unittests with several test cases require the node to be shut down before the next test case is started
        ros2py.shutdown(self.node)

    def request_point_cloud(self):
        request_data_goal = RequestData.Goal()
        request_data_goal.request_point_cloud = True

        response = ros2py.send_action_goal(self.node, self.request_data_client, request_data_goal)
        result = response.get_result()

        self.assertFalse(response.timeout(), msg="Requesting point cloud action timed out.")
        self.assertTrue(response.successful(), msg="Requesting point cloud action not successful.")
        self.assertEqual(result.error.code, 0, msg="Requesting point cloud not successful.")

    def retrieve_projected_depth_map(self):
        # Setup the view pose to be 90 degrees rotated w.r.t the original camera
        goal = TelecentricProjection.Goal()
        goal.view_pose = self.trafo
        goal.frame = FRAME
        goal.request_depth_image = True
        goal.include_results_in_response = True

        response = ros2py.send_action_goal(
            self.node, self.telecentric_projection_client, goal, timeout_sec=self.timeout
        )
        result = response.get_result()

        self.assertFalse(response.timeout(), msg="Requesting projected depth map timed out.")
        self.assertTrue(response.successful(), msg="Requesting projected depth map action not successful.")
        self.assertEqual(result.error.code, 0, msg="Requesting projected depth map not successful.")

        self.projected_depth_map = result.projected_depth_map

    def generate_line_estimate(self):
        bridge = CvBridge()

        # Message to cv image
        cv_image = bridge.imgmsg_to_cv2(self.projected_depth_map, desired_encoding="passthrough")

        # Convert it to a mono image (expressed in millimeters) via numpy
        mono8 = np.uint8(cv_image * 1000.0)
        cv.imwrite(IMAGE_PATH_MONO8, mono8)

        # Grey image to binary image
        _, binary_img = cv.threshold(mono8, 10, 255, cv.THRESH_BINARY)
        cv.imwrite(IMAGE_PATH_BINARY, binary_img)

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

    def retrieve_projected_point_cloud(self):
        goal = TelecentricProjection.Goal()
        goal.view_pose = self.trafo
        goal.frame = FRAME
        goal.include_results_in_response = True

        response = ros2py.send_action_goal(
            self.node, self.telecentric_projection_client, goal, timeout_sec=self.timeout
        )
        result = response.get_result()

        self.assertFalse(response.timeout(), msg="Requesting projected point cloud timed out.")
        self.assertTrue(response.successful(), msg="Requesting projected point cloud action not successful.")
        self.assertEqual(result.error.code, 0, msg="Requesting projected point cloud not successful.")

        self.projected_pc = result.projected_point_cloud

    def callback(self, msg):
        self.got_subscribed_cloud = True
        self.subscribed_cloud = msg

    def send_goal_with_publishing_point_cloud(self):
        goal = TelecentricProjection.Goal()
        goal.view_pose = self.trafo
        goal.frame = FRAME
        goal.publish_results = True

        response = ros2py.send_action_goal(
            self.node, self.telecentric_projection_client, goal, timeout_sec=self.timeout
        )
        result = response.get_result()

        self.assertFalse(response.timeout(), msg="Requesting publishing of point cloud timed out.")
        self.assertTrue(response.successful(), msg="Requesting publishing of point cloud action not successful.")
        self.assertEqual(result.error.code, 0, msg="Requesting publishing of point cloud not successful.")

    def test_found_line_paramters(self):
        self.assertAlmostEqual(self.line_parameters[0], 1.0, delta=0.05)
        self.assertAlmostEqual(self.line_parameters[1], 0.0, delta=0.05)

    def test_projected_point_cloud(self):
        cloud_points = list(pc2.read_points(self.projected_pc, skip_nans=True))
        self.assertTrue(len(cloud_points) != 0, msg=" The recieved point cloud is empty.")

    def test_subscribed_point_cloud(self):
        if not self.got_subscribed_cloud:
            ros2py.sleep(self.node, 4)
            self.assertTrue(self.got_subscribed_cloud, msg="Recieved no published point cloud!")
        cloud_points = list(pc2.read_points(self.subscribed_cloud, skip_nans=True))
        self.assertTrue(len(cloud_points) != 0, msg="The recieved point cloud is empty.")


def main():
    ros2py_testing.run_ros1_test("test_telecentric_projection", TestTelecentricProjection)


if __name__ == "__main__":
    main()
