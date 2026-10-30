#!/usr/bin/env python
import unittest

import ensenso_camera.ros2 as ros2py

from ensenso_camera_test.helper import Pose
import ensenso_camera_test.ros2_testing as ros2py_testing

LocatePattern = ros2py.import_action("ensenso_camera_msgs", "LocatePattern")

DATA_SET_FILEPATH = ros2py_testing.get_test_data_path("locate_pattern/pattern_pose.json")

FRAMES_CONTAIN_PATTERN = [True, False]


class TestLocatePattern(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_locate_pattern")
        self.locate_pattern_client = ros2py.create_action_client(self.node, "locate_pattern", LocatePattern)
        ros2py.wait_for_server(self.node, self.locate_pattern_client)

    def test_locate_pattern(self):
        for contains_pattern in FRAMES_CONTAIN_PATTERN:
            response = ros2py.send_action_goal(self.node, self.locate_pattern_client, LocatePattern.Goal())
            result = response.get_result()

            self.assertTrue(response.successful())
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

                reference_pattern_pose = Pose.from_json(DATA_SET_FILEPATH)
                estimated_pattern_pose = Pose.from_message(result.pattern_poses[0].pose)
                self.assertTrue(reference_pattern_pose.equals(estimated_pattern_pose))
            else:
                self.assertFalse(result.found_pattern)
                self.assertEqual(len(result.patterns), 0)
                self.assertEqual(len(result.pattern_poses), 0)


def main():
    ros2py_testing.run_ros1_test("test_locate_pattern", TestLocatePattern)


if __name__ == "__main__":
    main()
