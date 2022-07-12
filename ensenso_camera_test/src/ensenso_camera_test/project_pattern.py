#!/usr/bin/env python
import unittest

import ensenso_camera.ros2 as ros2py

from ensenso_camera_test.helper import ImagePoint
import ensenso_camera_test.ros2_testing as ros2py_testing

LocatePattern = ros2py.import_action("ensenso_camera_msgs", "LocatePattern")
ProjectPattern = ros2py.import_action("ensenso_camera_msgs", "ProjectPattern")


class TestProjectPattern(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_project_pattern")
        self.locate_pattern_client = ros2py.create_action_client(self.node, "locate_pattern", LocatePattern)
        self.project_pattern_client = ros2py.create_action_client(self.node, "project_pattern", ProjectPattern)

        clients = [self.locate_pattern_client, self.project_pattern_client]
        ros2py.wait_for_servers(self.node, clients)

    def test_project_pattern(self):
        response = ros2py.send_action_goal(self.node, self.locate_pattern_client, LocatePattern.Goal())
        location_result = response.get_result()

        self.assertTrue(response.successful())
        self.assertEqual(location_result.error.code, 0)

        self.assertTrue(location_result.found_pattern)
        self.assertEqual(len(location_result.patterns), 1)
        self.assertEqual(len(location_result.pattern_poses), 1)

        detected_pattern = location_result.patterns[0]
        detected_pattern_pose = location_result.pattern_poses[0].pose

        # Project the located pattern back into the camera. The result should be
        # the points that were detected in the image.
        goal = ProjectPattern.Goal()
        goal.grid_spacing = detected_pattern.grid_spacing
        goal.grid_size_x = detected_pattern.grid_size_x
        goal.grid_size_y = detected_pattern.grid_size_y
        goal.pattern_pose = detected_pattern_pose

        response = ros2py.send_action_goal(self.node, self.project_pattern_client, goal)
        projection_result = response.get_result()

        self.assertTrue(response.successful())
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


def main():
    ros2py_testing.run_ros1_test("test_project_pattern", TestProjectPattern)


if __name__ == "__main__":
    main()
