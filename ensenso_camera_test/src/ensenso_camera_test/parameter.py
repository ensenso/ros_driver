#!/usr/bin/env python
import unittest

import ensenso_camera.ros2 as ros2py

from ensenso_camera_msgs.msg import Parameter

from ensenso_camera_test.helper import RegionOfInterest
import ensenso_camera_test.ros2_testing as ros2py_testing


GetParameter = ros2py.import_action("ensenso_camera_msgs", "GetParameter")
SetParameter = ros2py.import_action("ensenso_camera_msgs", "SetParameter")


class TestParameter(unittest.TestCase):
    def setUp(self):
        self.node = ros2py.create_node("test_parameter")
        self.get_parameter_client = ros2py.create_action_client(self.node, "get_parameter", GetParameter)
        self.set_parameter_client = ros2py.create_action_client(self.node, "set_parameter", SetParameter)

        clients = [self.get_parameter_client, self.set_parameter_client]
        ros2py.wait_for_servers(self.node, clients)

    def get_parameter(self, keys, parameter_set=""):
        goal = GetParameter.Goal(parameter_set=parameter_set, keys=keys)
        response = ros2py.send_action_goal(self.node, self.get_parameter_client, goal)
        result = response.get_result()

        self.assertTrue(response.successful())
        self.assertEqual(result.error.code, 0)

        self.assertTrue(
            abs(ros2py_testing.to_sec(result.stamp) - ros2py_testing.to_sec(self.node.get_clock().now())) < 1
        )

        return result.results

    def set_parameter(self, parameters, parameter_set=""):
        goal = SetParameter.Goal(parameter_set=parameter_set, parameters=parameters)
        response = ros2py.send_action_goal(self.node, self.set_parameter_client, goal)
        result = response.get_result()

        self.assertTrue(response.successful())
        self.assertEqual(result.error.code, 0)

        return result.results

    def test_parameter(self):
        # We only test the trigger mode, because the other parameters are not
        # supported by a file camera.

        result = self.get_parameter([Parameter.TRIGGER_MODE])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.TRIGGER_MODE)
        self.assertEqual(result[0].string_value, "Software")

        # Set the parameter in a specific parameter set.
        result = self.set_parameter(
            [Parameter(key=Parameter.TRIGGER_MODE, string_value="Continuous")], parameter_set="test"
        )
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.TRIGGER_MODE)
        self.assertEqual(result[0].string_value, "Continuous")

        # We should be able to read the new value.
        result = self.get_parameter([Parameter.TRIGGER_MODE], parameter_set="test")
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.TRIGGER_MODE)
        self.assertEqual(result[0].string_value, "Continuous")

        # But it should still have the same value in the default parameter set.
        result = self.get_parameter([Parameter.TRIGGER_MODE])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.TRIGGER_MODE)
        self.assertEqual(result[0].string_value, "Software")

        # Test the special ROI parameter that does not map to an NxLib node.

        default_roi = RegionOfInterest([0, 0, 0], [0, 0, 0])
        test_roi = RegionOfInterest([1, 2, 3], [4, 5, 6])

        result = self.get_parameter([Parameter.REGION_OF_INTEREST])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.REGION_OF_INTEREST)
        self.assertTrue(RegionOfInterest.from_message(result[0].region_of_interest_value).equals(default_roi))

        roi_message = Parameter(key=Parameter.REGION_OF_INTEREST)
        test_roi.write_to_message(roi_message.region_of_interest_value)
        result = self.set_parameter([roi_message])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.REGION_OF_INTEREST)
        self.assertTrue(RegionOfInterest.from_message(result[0].region_of_interest_value).equals(test_roi))

        result = self.get_parameter([Parameter.REGION_OF_INTEREST])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.REGION_OF_INTEREST)
        self.assertTrue(RegionOfInterest.from_message(result[0].region_of_interest_value).equals(test_roi))


def main():
    ros2py_testing.run_ros1_test("test_parameter", TestParameter)


if __name__ == "__main__":
    main()
