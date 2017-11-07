#!/usr/bin/env python
import rospy
import rostest

import random
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import CalibrateWorkspaceAction, CalibrateWorkspaceGoal
from ensenso_camera_msgs.msg import LocatePatternAction, LocatePatternGoal

from geometry_msgs.msg import Pose, Point, Quaternion


class TestWorkspaceCalibration(unittest.TestCase):
    def setUp(self):
        self.calibration_client = actionlib.SimpleActionClient("calibrate_workspace", CalibrateWorkspaceAction)
        self.calibration_client.wait_for_server()
        self.locate_pattern_client = actionlib.SimpleActionClient("locate_pattern", LocatePatternAction)
        self.locate_pattern_client.wait_for_server()

    def test_workspace_calibration(self):
        for i in range(5):
            x = random.randint(-500, 500)
            y = random.randint(-500, 500)
            z = random.randint(-500, 500)
            pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))

            # Calibrate for the random pose.
            self.calibration_client.send_goal(CalibrateWorkspaceGoal(defined_pattern_pose=pose))
            self.calibration_client.wait_for_result()
            self.assertEqual(self.calibration_client.get_state(), GoalStatus.SUCCEEDED)
            self.assertEqual(self.calibration_client.get_result().error.code, 0)
            self.assertTrue(self.calibration_client.get_result().successful)

            # Locate the pattern. It should now be at the random pose.
            self.locate_pattern_client.send_goal(LocatePatternGoal())
            self.locate_pattern_client.wait_for_result()
            self.assertEqual(self.locate_pattern_client.get_state(), GoalStatus.SUCCEEDED)
            result = self.locate_pattern_client.get_result()
            self.assertEqual(result.error.code, 0)

            self.assertTrue(result.found_pattern)
            self.assertEqual(len(result.pattern_poses), 1)

            pose = result.pattern_poses[0].pose
            self.assertAlmostEqual(pose.position.x, x)
            self.assertAlmostEqual(pose.position.y, y)
            self.assertAlmostEqual(pose.position.z, z)


if __name__ == "__main__":
    try:
        rospy.init_node("test_workspace_calibration")
        rostest.rosrun("ensenso_camera_test", "test_workspace_calibration", TestWorkspaceCalibration)
    except rospy.ROSInterruptException:
        pass
