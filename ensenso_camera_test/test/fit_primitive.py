#!/usr/bin/env python
import rospy
import rostest

import unittest

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import FitPrimitiveAction, FitPrimitiveGoal, Primitive
from ensenso_camera_msgs.msg import RequestDataAction, RequestDataGoal

from helper import Point


# All used parameters with a length unit use meters as unit

# Input parameters for "fit_primitive" action
def get_cylinder_fit_primitive():
    primitive = Primitive()
    primitive.type = Primitive.CYLINDER
    primitive.inlier_threshold = 0.001
    primitive.count = 1
    primitive.inlier_fraction_in = 0.015
    primitive.min_radius = 0.095
    primitive.max_radius = 0.105
    return primitive


def get_sphere_fit_primitive():
    primitive = Primitive()
    primitive.type = Primitive.SPHERE
    primitive.inlier_threshold = 0.001
    primitive.count = 1
    primitive.inlier_fraction_in = 0.03
    primitive.min_radius = 0.048
    primitive.max_radius = 0.055
    return primitive


def get_plane_fit_primitive():
    primitive = Primitive()
    primitive.type = Primitive.PLANE
    primitive.inlier_threshold = 0.001
    primitive.count = 1
    primitive.inlier_fraction_in = 0.015
    primitive.min_radius = 0.095
    primitive.max_radius = 0.105
    return primitive


# Exact values of objects in the test scene
class SphereTestValues:
    def __init__(self):
        self.center = Point(0, 0.1, -0.1)
        self.radius = 0.05


class CylinderTestValues:
    def __init__(self):
        self.center = Point(0.0, -0.1, -0.25)
        self.radius = 0.1


class PlaneTestValues:
    def __init__(self):
        self.normal = Point(0, 0, -1.0)
        self.center = Point(0, 0, -0.06)


class TestFitPrimitive(unittest.TestCase):
    def setUp(self):
        self.fit_primitive_client = actionlib.SimpleActionClient("fit_primitive", FitPrimitiveAction)
        self.fit_primitive_client.wait_for_server()

        self.request_data_client = actionlib.SimpleActionClient("request_data", RequestDataAction)
        self.request_data_client.wait_for_server()

        request_data_goal = RequestDataGoal()
        request_data_goal.request_point_cloud = True
        self.request_data_client.send_goal(request_data_goal)
        self.request_data_client.wait_for_result()

        result = self.request_data_client.get_result()
        self.assertEqual(result.error.code, 0)

    def test_sphere(self):
        goal = FitPrimitiveGoal()
        primitive = get_sphere_fit_primitive()
        goal.primitives.append(primitive)

        result = self.get_result(goal)
        self.result_check(result)

    def test_plane(self):
        goal = FitPrimitiveGoal()
        primitive = get_plane_fit_primitive()
        goal.primitives.append(primitive)

        result = self.get_result(goal)
        self.result_check(result)

    def test_cylinder(self):
        goal = FitPrimitiveGoal()
        primitive = get_cylinder_fit_primitive()
        goal.primitives.append(primitive)

        result = self.get_result(goal)
        self.result_check(result)

    def test_primitive_combinations(self):
        goal = FitPrimitiveGoal()

        goal.primitives.append(get_sphere_fit_primitive())
        goal.primitives.append(get_cylinder_fit_primitive())
        goal.primitives.append(get_plane_fit_primitive())

        result = self.get_result(goal)
        self.result_check(result)

    def test_counts_of_object(self):
        goal = FitPrimitiveGoal()
        primitive = get_plane_fit_primitive()
        primitive.count = 3
        goal.primitives.append(primitive)

        result = self.get_result(goal)
        self.result_check(result)

    def result_check(self, result):
        delta = 0.005
        for primitive in result.primitives:
            if primitive.type == get_sphere_fit_primitive().type:
                sphere = SphereTestValues()
                self.assertAlmostEqual(primitive.center.x, sphere.center.x, delta=delta)
                self.assertAlmostEqual(primitive.center.y, sphere.center.y, delta=delta)
                self.assertAlmostEqual(primitive.center.z, sphere.center.z, delta=delta)
                self.assertAlmostEqual(primitive.radius, sphere.radius, delta=delta)
            elif primitive.type == get_cylinder_fit_primitive().type:
                cylinder = CylinderTestValues()
                self.assertAlmostEqual(primitive.center.y, cylinder.center.y, delta=delta)
                self.assertAlmostEqual(primitive.center.z, cylinder.center.y, delta=delta)
                self.assertAlmostEqual(primitive.radius, cylinder.radius, delta=delta)
            elif primitive.type == get_plane_fit_primitive().type:
                plane = PlaneTestValues()
                self.assertAlmostEqual(abs(primitive.normal.z), abs(plane.normal.z), delta=delta)
                self.assertAlmostEqual(primitive.center.z, plane.center.z, delta=delta)

    def get_result(self, goal):
        self.fit_primitive_client.send_goal(goal)
        self.fit_primitive_client.wait_for_result()
        self.assertEqual(self.fit_primitive_client.get_state(), GoalStatus.SUCCEEDED)
        result = self.fit_primitive_client.get_result()
        self.assertEqual(result.error.code, 0)
        return result


if __name__ == "__main__":
    try:
        rospy.init_node("test_fit_primitive")
        rostest.rosrun("ensenso_camera_test", "test_fit_primitive", TestFitPrimitive)
    except rospy.ROSInterruptException:
        pass
