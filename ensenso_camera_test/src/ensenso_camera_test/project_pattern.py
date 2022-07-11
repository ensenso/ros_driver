#!/usr/bin/env python
import rospy
import rostest

import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import LocatePatternAction, LocatePatternGoal
from ensenso_camera_msgs.msg import ProjectPatternAction, ProjectPatternGoal

from helper import ImagePoint


class TestProjectPattern(unittest.TestCase):
    def setUp(self):
        self.locate_pattern_client = actionlib.SimpleActionClient("locate_pattern", LocatePatternAction)
        self.locate_pattern_client.wait_for_server()
        self.project_pattern_client = actionlib.SimpleActionClient("project_pattern", ProjectPatternAction)
        self.project_pattern_client.wait_for_server()

    def test_project_pattern(self):
        self.locate_pattern_client.send_goal(LocatePatternGoal())
        self.locate_pattern_client.wait_for_result()
        self.assertEqual(self.locate_pattern_client.get_state(), GoalStatus.SUCCEEDED)
        location_result = self.locate_pattern_client.get_result()
        self.assertEqual(location_result.error.code, 0)

        self.assertTrue(location_result.found_pattern)
        self.assertEqual(len(location_result.patterns), 1)
        self.assertEqual(len(location_result.pattern_poses), 1)

        detected_pattern = location_result.patterns[0]
        detected_pattern_pose = location_result.pattern_poses[0].pose

        # Project the located pattern back into the camera. The result should be
        # the points that were detected in the image.

        goal = ProjectPatternGoal()
        goal.grid_spacing = detected_pattern.grid_spacing
        goal.grid_size_x = detected_pattern.grid_size_x
        goal.grid_size_y = detected_pattern.grid_size_y
        goal.pattern_pose = detected_pattern_pose

        self.project_pattern_client.send_goal(goal)
        self.project_pattern_client.wait_for_result()

        self.assertEqual(self.project_pattern_client.get_state(), GoalStatus.SUCCEEDED)
        projection_result = self.project_pattern_client.get_result()
        self.assertEqual(projection_result.error.code, 0)

        self.assertEqual(projection_result.pattern_is_visible, True)
        self.assertEqual(len(detected_pattern.left_points), len(projection_result.left_points))
        self.assertEqual(len(detected_pattern.right_points), len(projection_result.right_points))

        reference_points = map(
            lambda p: ImagePoint.from_message(p), detected_pattern.left_points + detected_pattern.right_points
        )
        projected_points = map(
            lambda p: ImagePoint.from_message(p), projection_result.left_points + projection_result.right_points
        )
        for reference_point, projected_point in zip(reference_points, projected_points):
            self.assertTrue(reference_point.equals(projected_point))


if __name__ == "__main__":
    try:
        rospy.init_node("test_project_pattern")
        rostest.rosrun("ensenso_camera_test", "test_project_pattern", TestProjectPattern)
    except rospy.ROSInterruptException:
        pass
