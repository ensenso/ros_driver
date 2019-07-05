#!/usr/bin/env python
import rospy
import rostest

import os
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import LocatePatternAction, LocatePatternGoal

from helper import Pose

DATA_SET_DIRECTORY = "../data/locate_pattern/"

FRAMES_CONTAIN_PATTERN = [True, False]


class TestLocatePattern(unittest.TestCase):
    def setUp(self):
        self.locate_pattern_client = actionlib.SimpleActionClient("locate_pattern", LocatePatternAction)
        self.locate_pattern_client.wait_for_server()

    def test_locate_pattern(self):
        for contains_pattern in FRAMES_CONTAIN_PATTERN:
            self.locate_pattern_client.send_goal(LocatePatternGoal())
            self.locate_pattern_client.wait_for_result()
            self.assertEqual(self.locate_pattern_client.get_state(), GoalStatus.SUCCEEDED)
            result = self.locate_pattern_client.get_result()
            self.assertEqual(result.error.code, 0)

            if contains_pattern:
                self.assertTrue(result.found_pattern)
                self.assertEqual(len(result.patterns), 1)
                self.assertEqual(len(result.pattern_poses), 1)

                self.assertNotEqual(result.frame, "")

                pattern = result.patterns[0]
                self.assertEqual(pattern.thickness, 0.001)
                self.assertEqual(pattern.grid_spacing, 0.01875)
                self.assertEqual(pattern.grid_size_x, 7)
                self.assertEqual(pattern.grid_size_y, 7)
                self.assertEqual(len(pattern.left_points), 49)
                self.assertEqual(len(pattern.right_points), 49)

                self.assertNotEqual(result.pattern_poses[0].header.stamp, 0)
                self.assertNotEqual(result.pattern_poses[0].header.frame_id, "")

                reference_pattern_pose = Pose.from_json(os.path.join(DATA_SET_DIRECTORY, "pattern_pose.json"))
                estimated_pattern_pose = Pose.from_message(result.pattern_poses[0].pose)
                self.assertTrue(reference_pattern_pose.equals(estimated_pattern_pose))
            else:
                self.assertFalse(result.found_pattern)
                self.assertEqual(len(result.patterns), 0)
                self.assertEqual(len(result.pattern_poses), 0)


if __name__ == "__main__":
    try:
        rospy.init_node("test_locate_pattern")
        rostest.rosrun("ensenso_camera_test", "test_locate_pattern", TestLocatePattern)
    except rospy.ROSInterruptException:
        pass
