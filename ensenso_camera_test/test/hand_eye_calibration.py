#!/usr/bin/env python
import rospy
import rostest

import glob
import os
import unittest

import actionlib
import tf

from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import CalibrateHandEyeAction, CalibrateHandEyeGoal

from helper import Pose

DATA_SET_DIRECTORY = "../data/hand_eye_calibration/"

ROBOT_BASE_FRAME = "robot_base"
ROBOT_WRIST_FRAME = "robot_wrist"

# The indices of the data set frames that do not contain any visible patterns.
FRAMES_WITHOUT_PATTERNS = [2, 3]


class TestHandEyeCalibration(unittest.TestCase):
    def setUp(self):
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.calibration_client = actionlib.SimpleActionClient("calibrate_hand_eye", CalibrateHandEyeAction)
        self.calibration_client.wait_for_server()

    def assert_goal_successful(self):
        self.assertEqual(self.calibration_client.get_state(), GoalStatus.SUCCEEDED)
        self.assertEqual(self.calibration_client.get_result().error.code, 0)

    def send_calibration_goal(self, goal, feedback_cb=None, wait=True):
        self.calibration_client.send_goal(goal, feedback_cb=feedback_cb)

        if wait:
            self.calibration_client.wait_for_result()
            self.assert_goal_successful()
            self.assertEqual(self.calibration_client.get_result().command, goal.command)

    def _broadcast_tf(self, pose, child_frame, parent_frame):
        self.tf_broadcaster.sendTransform(pose.position, pose.orientation, rospy.Time.now(), child_frame, parent_frame)

    def test_hand_eye_calibration(self):
        self.send_calibration_goal(CalibrateHandEyeGoal(command=CalibrateHandEyeGoal.RESET))

        result = self.calibration_client.get_result()
        self.assertEqual(result.command, CalibrateHandEyeGoal.RESET)
        self.assertEqual(result.error_message, "")
        self.assertEqual(result.error.code, 0)

        link_files = sorted(glob.glob(os.path.join(DATA_SET_DIRECTORY, "links/links*")))
        self.assertEqual(len(link_files), 22)
        for i, link_file in enumerate(link_files):
            current_robot_pose = Pose.from_json(link_file)

            self._broadcast_tf(current_robot_pose, ROBOT_WRIST_FRAME, ROBOT_BASE_FRAME)
            self.send_calibration_goal(CalibrateHandEyeGoal(command=CalibrateHandEyeGoal.CAPTURE_PATTERN))

            result = self.calibration_client.get_result()
            self.assertEqual(result.command, CalibrateHandEyeGoal.CAPTURE_PATTERN)
            self.assertEqual(result.error_message, "")
            self.assertEqual(result.error.code, 0)

            if not result.found_pattern:
                self.assertIn(i, FRAMES_WITHOUT_PATTERNS)
            else:
                self.assertEqual(result.pattern.thickness, 0.001)
                self.assertEqual(result.pattern.grid_spacing, 0.01875)
                self.assertEqual(result.pattern.grid_size_x, 7)
                self.assertEqual(result.pattern.grid_size_y, 7)
                self.assertEqual(len(result.pattern.left_points), 49)
                self.assertEqual(len(result.pattern.right_points), 49)

                robot_pose = Pose.from_message(result.robot_pose)
                self.assertTrue(robot_pose.equals(current_robot_pose))

                pattern_pose = Pose.from_message(result.pattern_pose)
                self.assertTrue(all([x != 0 for x in pattern_pose.position]))

        self.last_feedback = None

        def on_feedback(feedback):
            self.last_feedback = feedback

        self.send_calibration_goal(CalibrateHandEyeGoal(command=CalibrateHandEyeGoal.START_CALIBRATION),
                                   feedback_cb=on_feedback)

        self.assertIsNotNone(self.last_feedback)
        self.assertNotEqual(self.last_feedback.number_of_iterations, 0)
        self.assertNotEqual(self.last_feedback.reprojection_error, 0)

        result = self.calibration_client.get_result()
        self.assertEqual(result.command, CalibrateHandEyeGoal.START_CALIBRATION)
        self.assertEqual(result.error_message, "")
        self.assertEqual(result.error.code, 0)

        self.assertNotEqual(result.calibration_time, 0)
        self.assertNotEqual(result.number_of_iterations, 0)
        self.assertNotEqual(result.reprojection_error, 0)

        final_link = Pose.from_json(os.path.join(DATA_SET_DIRECTORY, "final_link.json")).inverse()
        final_pattern_pose = Pose.from_json(os.path.join(DATA_SET_DIRECTORY, "final_pattern_pose.json"))
        self.assertTrue(final_link.equals(Pose.from_message(result.link)))
        self.assertTrue(final_pattern_pose.equals(Pose.from_message(result.pattern_pose)))


if __name__ == "__main__":
    try:
        rospy.init_node("test_hand_eye_calibration")
        rostest.rosrun("ensenso_camera_test", "test_hand_eye_calibration", TestHandEyeCalibration)
    except rospy.ROSInterruptException:
        pass
