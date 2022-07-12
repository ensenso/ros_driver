#!/usr/bin/env python
import glob
import os
import unittest

import ensenso_camera.ros2 as ros2py

from ensenso_camera_test.helper import Pose
import ensenso_camera_test.ros2_testing as ros2py_testing

CalibrateHandEye = ros2py.import_action("ensenso_camera_msgs", "CalibrateHandEye")

DATA_SET_DIRECTORY = ros2py_testing.get_test_data_path("hand_eye_calibration/")

ROBOT_BASE_FRAME = "robot_base"
ROBOT_WRIST_FRAME = "robot_wrist"

# The indices of the data set frames that do not contain any visible patterns.
FRAMES_WITHOUT_PATTERNS = [2, 3]


class TestHandEyeCalibration(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_hand_eye_calibration")
        self.tf_broadcaster = ros2py_testing.create_tf_broadcaster(self.node)
        self.calibration_client = ros2py.create_action_client(self.node, "calibrate_hand_eye", CalibrateHandEye)
        success = ros2py.wait_for_server(
            self.node, self.calibration_client, timeout_sec=ros2py_testing.TEST_TIMEOUT, exit=False
        )
        self.assertTrue(success, msg="Timeout reached for servers.")

    def assert_goal_response(self, response, goal):
        result = response.get_result()
        self.assertTrue(response.successful())
        self.assertEqual(result.error.code, 0)
        self.assertEqual(result.command, goal.command)

    def send_calibration_goal(self, goal, feedback_callback=None):
        response = ros2py.send_action_goal(self.node, self.calibration_client, goal, feedback_callback)
        self.assert_goal_response(response, goal)
        return response

    def _broadcast_tf(self, pose, child_frame, parent_frame):
        timestamp = self.node.get_clock().now()
        ros2py_testing.send_tf_transform(self.tf_broadcaster, pose, timestamp, child_frame, parent_frame)

    def test_hand_eye_calibration(self):
        response = self.send_calibration_goal(CalibrateHandEye.Goal(command=CalibrateHandEye.Goal.RESET))

        result = response.get_result()
        self.assertEqual(result.command, CalibrateHandEye.Goal.RESET)
        self.assertEqual(result.error_message, "")
        self.assertEqual(result.error.code, 0)

        link_files = sorted(glob.glob(os.path.join(DATA_SET_DIRECTORY, "links/links*")))
        self.assertEqual(len(link_files), 22)
        for i, link_file in enumerate(link_files):
            current_robot_pose = Pose.from_json(link_file)

            self._broadcast_tf(current_robot_pose, ROBOT_WRIST_FRAME, ROBOT_BASE_FRAME)
            response = self.send_calibration_goal(CalibrateHandEye.Goal(command=CalibrateHandEye.Goal.CAPTURE_PATTERN))

            result = response.get_result()
            self.assertEqual(result.command, CalibrateHandEye.Goal.CAPTURE_PATTERN)
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

        @ros2py_testing.feedback_callback
        def on_feedback(feedback):
            self.last_feedback = feedback

        goal = CalibrateHandEye.Goal(command=CalibrateHandEye.Goal.START_CALIBRATION)
        response = self.send_calibration_goal(goal, feedback_callback=on_feedback)

        self.assertIsNotNone(self.last_feedback)
        self.assertNotEqual(self.last_feedback.number_of_iterations, 0)
        self.assertNotEqual(self.last_feedback.residual, 0)

        result = response.get_result()
        self.assertEqual(result.command, CalibrateHandEye.Goal.START_CALIBRATION)
        self.assertEqual(result.error_message, "")
        self.assertEqual(result.error.code, 0)

        self.assertNotEqual(result.calibration_time, 0)
        self.assertNotEqual(result.number_of_iterations, 0)
        self.assertNotEqual(result.residual, 0)

        final_link = Pose.from_json(os.path.join(DATA_SET_DIRECTORY, "final_link.json")).inverse()
        final_pattern_pose = Pose.from_json(os.path.join(DATA_SET_DIRECTORY, "final_pattern_pose.json"))
        self.assertTrue(final_link.equals(Pose.from_message(result.link)))
        self.assertTrue(final_pattern_pose.equals(Pose.from_message(result.pattern_pose)))


def main():
    ros2py_testing.run_ros1_test("test_hand_eye_calibration", TestHandEyeCalibration)


if __name__ == "__main__":
    main()
