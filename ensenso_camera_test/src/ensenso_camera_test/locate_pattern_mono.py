#!/usr/bin/env python
import rospy
import rostest

import os
import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import LocatePatternMonoAction, LocatePatternMonoGoal

from helper import Pose

DATA_SET_DIRECTORY = "../data/locate_pattern_mono/"


class TestLocatePattern(unittest.TestCase):
    def setUp(self):
        self.locate_pattern_client = actionlib.SimpleActionClient("locate_pattern", LocatePatternMonoAction)
        self.locate_pattern_client.wait_for_server()

    def test_locate_pattern(self):
        self.locate_pattern_client.send_goal(LocatePatternMonoGoal())
        self.locate_pattern_client.wait_for_result()
        self.assertEqual(self.locate_pattern_client.get_state(), GoalStatus.SUCCEEDED)
        result = self.locate_pattern_client.get_result()

        self.assertEqual(result.error.code, 0)

        self.assertTrue(result.found_pattern)
        self.assertEqual(len(result.mono_patterns), 1)
        self.assertEqual(len(result.mono_pattern_poses), 1)

        self.assertNotEqual(result.frame, "")

        pattern = result.mono_patterns[0]

        self.assertEqual(pattern.grid_spacing, 0.03625)
        self.assertEqual(pattern.grid_size_x, 7)
        self.assertEqual(pattern.grid_size_y, 7)
        self.assertEqual(len(pattern.points), 49)

        self.assertNotEqual(result.mono_pattern_poses[0].header.stamp, 0)
        self.assertNotEqual(result.mono_pattern_poses[0].header.frame_id, "")

        reference_pattern_pose = Pose.from_json(os.path.join(DATA_SET_DIRECTORY, "pattern_pose.json"))
        estimated_pattern_pose = Pose.from_message(result.mono_pattern_poses[0].pose)
        self.assertTrue(reference_pattern_pose.equals(estimated_pattern_pose))


if __name__ == "__main__":
    try:
        rospy.init_node("test_locate_pattern_mono")
        rostest.rosrun("ensenso_camera_test", "test_locate_pattern_mono", TestLocatePattern)
    except rospy.ROSInterruptException:
        pass
