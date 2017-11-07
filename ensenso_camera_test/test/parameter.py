#!/usr/bin/env python
import rospy
import rostest

import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import GetParameterAction, GetParameterGoal
from ensenso_camera_msgs.msg import SetParameterAction, SetParameterGoal

from ensenso_camera_msgs.msg import Parameter

from helper import RegionOfInterest


class TestParameter(unittest.TestCase):
    def setUp(self):
        self.get_parameter_client = actionlib.SimpleActionClient("get_parameter", GetParameterAction)
        self.get_parameter_client.wait_for_server()
        self.set_parameter_client = actionlib.SimpleActionClient("set_parameter", SetParameterAction)
        self.set_parameter_client.wait_for_server()

    def get_parameter(self, keys, parameter_set=""):
        self.get_parameter_client.send_goal(GetParameterGoal(parameter_set=parameter_set, keys=keys))
        self.get_parameter_client.wait_for_result()

        self.assertEqual(self.get_parameter_client.get_state(), GoalStatus.SUCCEEDED)
        self.assertEqual(self.get_parameter_client.get_result().error.code, 0)

        result = self.get_parameter_client.get_result()
        self.assertTrue(abs(result.stamp.to_sec() - rospy.Time.now().to_sec()) < 1)

        return result.results

    def set_parameter(self, parameters, parameter_set=""):
        self.set_parameter_client.send_goal(SetParameterGoal(parameter_set=parameter_set, parameters=parameters))
        self.set_parameter_client.wait_for_result()

        self.assertEqual(self.set_parameter_client.get_state(), GoalStatus.SUCCEEDED)
        self.assertEqual(self.set_parameter_client.get_result().error.code, 0)

        return self.set_parameter_client.get_result().results

    def test_parameter(self):
        # We only test the trigger mode, because the other parameters are not
        # supported by a file camera.

        result = self.get_parameter([Parameter.TRIGGER_MODE])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].key, Parameter.TRIGGER_MODE)
        self.assertEqual(result[0].string_value, "Software")

        # Set the parameter in a specific parameter set.
        result = self.set_parameter([Parameter(key=Parameter.TRIGGER_MODE, string_value="Continuous")],
                                    parameter_set="test")
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


if __name__ == "__main__":
    try:
        rospy.init_node("test_parameter")
        rostest.rosrun("ensenso_camera_test", "test_parameter", TestParameter)
    except rospy.ROSInterruptException:
        pass
