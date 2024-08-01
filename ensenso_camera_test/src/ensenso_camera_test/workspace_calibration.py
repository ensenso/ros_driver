#!/usr/bin/env python
import random
import unittest

import ensenso_camera.ros2 as ros2py

from ensenso_camera_msgs.msg import Parameter

import ensenso_camera_test.ros2_testing as ros2py_testing

from geometry_msgs.msg import Pose, Point, Quaternion

CalibrateWorkspace = ros2py.import_action("ensenso_camera_msgs", "CalibrateWorkspace")
LocatePattern = ros2py.import_action("ensenso_camera_msgs", "LocatePattern")
SetParameter = ros2py.import_action("ensenso_camera_msgs", "SetParameter")

tolerance = 0.001


class TestWorkspaceCalibration(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_workspace_calibration")
        self.calibration_client = ros2py.create_action_client(self.node, "calibrate_workspace", CalibrateWorkspace)
        self.locate_pattern_client = ros2py.create_action_client(self.node, "locate_pattern", LocatePattern)
        self.set_parameter_client = ros2py.create_action_client(self.node, "set_parameter", SetParameter)
        ros2py.wait_for_server(self.node, self.locate_pattern_client)

    def disable_follow_link(self):
        parameter = Parameter(key=Parameter.FOLLOW_LINK, string_value="Disabled")
        goal = SetParameter.Goal(parameters=[parameter])
        response = ros2py.send_action_goal(self.node, self.set_parameter_client, goal)

        result = response.get_result()
        self.assertTrue(response.successful())
        self.assertEqual(result.error.code, 0)

    def test_workspace_calibration(self):
        for _ in range(5):
            x = random.randint(-500, 500)
            y = random.randint(-500, 500)
            z = random.randint(-500, 500)

            p = Point(x=float(x), y=float(y), z=float(z))

            q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            pose = Pose(position=p, orientation=q)

            # Calibrate for the random pose.
            goal = CalibrateWorkspace.Goal(defined_pattern_pose=pose)
            response = ros2py.send_action_goal(self.node, self.calibration_client, goal)
            result = response.get_result()

            self.assertTrue(response.successful())
            self.assertEqual(result.error.code, 0)
            self.assertTrue(result.successful)

            self.disable_follow_link()

            # Locate the pattern. It should now be at the random pose.
            response = ros2py.send_action_goal(self.node, self.locate_pattern_client, LocatePattern.Goal())
            result = response.get_result()

            self.assertTrue(response.successful())
            self.assertEqual(result.error.code, 0)

            self.assertTrue(result.found_pattern)
            self.assertEqual(len(result.pattern_poses), 1)

            pose = result.pattern_poses[0].pose
            self.assertTrue(abs(pose.position.x - x) < tolerance)
            self.assertTrue(abs(pose.position.y - y) < tolerance)
            self.assertTrue(abs(pose.position.z - z) < tolerance)


def main():
    ros2py_testing.run_ros1_test("test_workspace_calibration", TestWorkspaceCalibration)


if __name__ == "__main__":
    main()
