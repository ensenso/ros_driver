#!/usr/bin/env python
import unittest

import ensenso_camera.ros2 as ros2py

from ensenso_camera_test.helper import Pose
import ensenso_camera_test.ros2_testing as ros2py_testing

LocatePatternMono = ros2py.import_action("ensenso_camera_msgs", "LocatePatternMono")

DATA_SET_FILEPATH = ros2py_testing.get_test_data_path("locate_pattern_mono/pattern_pose.json")


class TestLocatePatternMono(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_locate_pattern_mono")
        self.locate_pattern_client = ros2py.create_action_client(self.node, "locate_pattern", LocatePatternMono)
        ros2py.wait_for_server(self.node, self.locate_pattern_client)

    def test_locate_pattern(self):
        response = ros2py.send_action_goal(self.node, self.locate_pattern_client, LocatePatternMono.Goal())
        result = response.get_result()

        self.assertTrue(response.successful())
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

        reference_pattern_pose = Pose.from_json(DATA_SET_FILEPATH)
        estimated_pattern_pose = Pose.from_message(result.mono_pattern_poses[0].pose)
        self.assertTrue(reference_pattern_pose.equals(estimated_pattern_pose))


def main():
    ros2py_testing.run_ros1_test("test_locate_pattern_mono", TestLocatePatternMono)


if __name__ == "__main__":
    main()
