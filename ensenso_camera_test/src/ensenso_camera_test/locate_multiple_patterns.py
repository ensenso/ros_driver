#!/usr/bin/env python
import unittest

import ensenso_camera.ros2 as ros2py

import ensenso_camera_test.ros2_testing as ros2py_testing

LocatePattern = ros2py.import_action("ensenso_camera_msgs", "LocatePattern")


class TestLocateMultiplePatterns(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_locate_multiple_patterns")
        self.locate_pattern_client = ros2py.create_action_client(self.node, "locate_pattern", LocatePattern)
        ros2py.wait_for_server(self.node, self.locate_pattern_client)

    def test_locate_multiple_patterns(self):
        response = ros2py.send_action_goal(self.node, self.locate_pattern_client, LocatePattern.Goal())
        result = response.get_result()

        self.assertTrue(response.successful())
        self.assertEqual(result.error.code, 0)

        self.assertTrue(result.found_pattern)
        self.assertEqual(len(result.patterns), 2)
        self.assertEqual(len(result.pattern_poses), 2)


def main():
    ros2py_testing.run_ros1_test("test_locate_multiple_patterns", TestLocateMultiplePatterns)


if __name__ == "__main__":
    main()
