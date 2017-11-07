#!/usr/bin/env python
import rospy
import rostest

import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import LocatePatternAction, LocatePatternGoal


class TestLocatePattern(unittest.TestCase):
    def setUp(self):
        self.locate_pattern_client = actionlib.SimpleActionClient("locate_pattern", LocatePatternAction)
        self.locate_pattern_client.wait_for_server()

    def test_locate_multiple_patterns(self):
        self.locate_pattern_client.send_goal(LocatePatternGoal())
        self.locate_pattern_client.wait_for_result()
        self.assertEqual(self.locate_pattern_client.get_state(), GoalStatus.SUCCEEDED)
        result = self.locate_pattern_client.get_result()
        self.assertEqual(result.error.code, 0)

        self.assertTrue(result.found_pattern)
        self.assertEqual(len(result.patterns), 2)
        self.assertEqual(len(result.pattern_poses), 2)


if __name__ == "__main__":
    try:
        rospy.init_node("test_locate_multiple_patterns")
        rostest.rosrun("ensenso_camera_test", "test_locate_multiple_patterns", TestLocatePattern)
    except rospy.ROSInterruptException:
        pass
