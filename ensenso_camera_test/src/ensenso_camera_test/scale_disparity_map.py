#!/usr/bin/env python
import numpy as np
import unittest

from scipy.spatial.transform import Rotation

import ensenso_camera.ros2 as ros2py

from ensenso_camera_msgs.msg import Parameter

import ensenso_camera_test.ros2_testing as ros2py_testing

from sensor_msgs.msg import PointCloud2, PointField


pc2 = ros2py_testing.import_point_cloud2()

RequestData = ros2py.import_action("ensenso_camera_msgs", "RequestData")
SetParameter = ros2py.import_action("ensenso_camera_msgs", "SetParameter")


# In ROS2 the depth_image_proc/point_cloud_xyz node has a fixed QOS profile and the message is not received
# after a single request, hence we request the data until we received a point cloud from the node.
MAX_TRIES = 10

# The corresponding test launch file opens a camera that is linked to "Workspace".
TARGET_FRAME = "Workspace"


class TestScaleDisparityMap(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_scale_disparity_map")

        self.tf_buffer = ros2py_testing.create_tf_buffer()
        self.tf_listener = ros2py_testing.create_tf_listener(self.tf_buffer, self.node)

        self.request_data_client = ros2py.create_action_client(self.node, "request_data", RequestData)
        self.set_parameter_client = ros2py.create_action_client(self.node, "set_parameter", SetParameter)

        clients = [self.request_data_client, self.set_parameter_client]
        success = ros2py.wait_for_servers(self.node, clients, timeout_sec=ros2py_testing.TEST_TIMEOUT, exit=False)
        self.assertTrue(success, msg="Timeout reached for servers.")

        self.point_cloud_ens = None
        self.point_cloud_dip = None

        # The Ensenso point cloud and the reference point cloud calculated by depth_image_proc/point_cloud_xyz
        self.pc_subscriber1 = ros2py.create_subscription(self.node, PointCloud2, "point_cloud", self.callback_ens)
        self.pc_subscriber2 = ros2py.create_subscription(
            self.node, PointCloud2, "point_cloud_dip", self.callback_dip, ros2py.get_qos_profile("point_cloud_dip")
        )

    def request_data(self):
        request_data_goal = RequestData.Goal()
        request_data_goal.request_rectified_images = True
        request_data_goal.request_disparity_map = True
        request_data_goal.request_depth_image = True
        request_data_goal.request_point_cloud = True
        request_data_goal.publish_results = True

        response = ros2py.send_action_goal(self.node, self.request_data_client, request_data_goal)
        result = response.get_result()

        self.assertFalse(response.timeout(), msg="Requesting data action timed out.")
        self.assertTrue(response.successful(), msg="Requesting data action not successful.")
        self.assertEqual(result.error.code, 0, msg="Requesting data not successful.")

    def callback_ens(self, msg):
        self.point_cloud_ens = msg

    def callback_dip(self, msg):
        self.point_cloud_dip = msg

    def transform_point_cloud_to_target_frame(self, point_cloud, target_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                point_cloud.header.frame_id,
                point_cloud.header.stamp,
            )
        except Exception as e:
            self.node.get_logger().error(e)

        # See https://robotics.stackexchange.com/questions/109924/
        t = np.eye(4)
        q = tf.transform.rotation
        x = tf.transform.translation
        t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t[:3, 3] = [x.x, x.y, x.z]

        points_msg = list(pc2.read_points(point_cloud))
        points_map = np.ones((len(points_msg), 4))
        points_map[:, :3] = points_msg
        points_map = np.dot(t, points_map.T).T

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pcl_map_header = point_cloud.header
        pcl_map_header.frame_id = "map"
        return pc2.create_cloud(pcl_map_header, fields, points_map[:, :3])

    def set_parameters(self, parameters=[]):
        goal = SetParameter.Goal(parameters=parameters)
        response = ros2py.send_action_goal(self.node, self.set_parameter_client, goal)

        result = response.get_result()
        self.assertTrue(response.successful())
        self.assertEqual(result.error.code, 0)

    def test_scale_disparity_map(self):
        # Enable disparity map scaling.
        self.set_parameters([Parameter(key=Parameter.SCALING, float_value=0.5)])

        # Wait until we received a reference point cloud.
        tries = 0
        while self.point_cloud_dip is None and tries < MAX_TRIES:
            self.request_data()
            ros2py.sleep(self.node, 2)
            tries += 1

        self.assertIsNotNone(self.point_cloud_ens, msg="Received no Ensenso point cloud")
        self.assertIsNotNone(self.point_cloud_dip, msg="Received no reference point cloud")

        frame = self.point_cloud_ens.header.frame_id
        self.assertEqual(
            frame,
            TARGET_FRAME,
            msg=f"Expected Ensenso point cloud in frame '{TARGET_FRAME}', got '{frame}'!",
        )

        # Since cmdComputeDisparityMap returns the point cloud in the camera's linked coordinate system, the reference
        # point cloud has to be transformed to that coordinate system as well.
        self.point_cloud_dip = self.transform_point_cloud_to_target_frame(self.point_cloud_dip, TARGET_FRAME)

        points_ens = list(pc2.read_points(self.point_cloud_ens, skip_nans=True))
        points_dip = list(pc2.read_points(self.point_cloud_dip, skip_nans=True))
        self.assertGreater(len(points_ens), 0, msg="The Ensenso point cloud is empty")
        self.assertGreater(len(points_dip), 0, msg="The reference point cloud is empty")

        def average_values(points, axis=0):
            return np.average(np.array(points)[:, axis : axis + 1]) * 1000.0

        self.assertAlmostEqual(
            average_values(points_ens, axis=0),
            average_values(points_dip, axis=0),
            places=3,
            msg="x values differ",
        )

        self.assertAlmostEqual(
            average_values(points_ens, axis=1),
            average_values(points_dip, axis=1),
            places=3,
            msg="y values differ",
        )

        self.assertAlmostEqual(
            average_values(points_ens, axis=2),
            average_values(points_dip, axis=2),
            places=3,
            msg="z values differ",
        )


def main():
    ros2py_testing.run_ros1_test("test_scale_disparity_map", TestScaleDisparityMap)


if __name__ == "__main__":
    main()
