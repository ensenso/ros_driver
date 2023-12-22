#!/usr/bin/env python
import unittest

import ensenso_camera.ros2 as ros2py

from ensenso_camera_msgs.msg import Primitive

from ensenso_camera_test.helper import Point
import ensenso_camera_test.ros2_testing as ros2py_testing

FitPrimitive = ros2py.import_action("ensenso_camera_msgs", "FitPrimitive")
RequestData = ros2py.import_action("ensenso_camera_msgs", "RequestData")

INLIER_THRESHOLD = 0.005


# All used parameters with a length unit use meters as unit


def get_cylinder_fit_primitive():
    primitive = Primitive()
    primitive.type = Primitive.CYLINDER
    primitive.inlier_threshold = INLIER_THRESHOLD
    primitive.count = 1
    primitive.inlier_fraction_in = 0.015
    primitive.min_radius = 0.095
    primitive.max_radius = 0.105
    return primitive


def get_sphere_fit_primitive():
    primitive = Primitive()
    primitive.type = Primitive.SPHERE
    primitive.inlier_threshold = INLIER_THRESHOLD
    primitive.count = 1
    primitive.inlier_fraction_in = 0.03
    primitive.min_radius = 0.048
    primitive.max_radius = 0.055
    return primitive


def get_plane_fit_primitive():
    primitive = Primitive()
    primitive.type = Primitive.PLANE
    primitive.inlier_threshold = INLIER_THRESHOLD
    primitive.count = 1
    primitive.inlier_fraction_in = 0.015
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
        self.node = ros2py.create_node("test_fit_primitive")
        self.fit_primitive_client = ros2py.create_action_client(self.node, "fit_primitive", FitPrimitive)
        self.request_data_client = ros2py.create_action_client(self.node, "request_data", RequestData)

        clients = [self.fit_primitive_client, self.request_data_client]
        success = ros2py.wait_for_servers(self.node, clients, timeout_sec=ros2py_testing.TEST_TIMEOUT, exit=False)
        self.assertTrue(success, msg="Timeout reached for servers.")

        request_data_goal = RequestData.Goal()
        request_data_goal.request_point_cloud = True
        response = ros2py.send_action_goal(self.node, self.request_data_client, request_data_goal)

        result = response.get_result()
        self.assertEqual(result.error.code, 0)

    def tearDown(self):
        ros2py.shutdown(self.node)

    def test_sphere(self):
        goal = FitPrimitive.Goal()
        primitive = get_sphere_fit_primitive()
        goal.primitives.append(primitive)
        result = self.get_result(goal)
        self.result_check(result)

    def test_plane(self):
        goal = FitPrimitive.Goal()
        primitive = get_plane_fit_primitive()
        goal.primitives.append(primitive)

        result = self.get_result(goal)
        self.result_check(result)

    def test_cylinder(self):
        goal = FitPrimitive.Goal()
        primitive = get_cylinder_fit_primitive()
        goal.primitives.append(primitive)

        result = self.get_result(goal)
        self.result_check(result)

    def result_check(self, result):
        delta = INLIER_THRESHOLD * 2
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
        response = ros2py.send_action_goal(self.node, self.fit_primitive_client, goal)
        self.assertTrue(response.successful(), msg="Fit primitive action has not been successful.")
        error = response.get_result().error
        self.assertEqual(error.code, 0, msg=ros2py.format_error(error, note="Fit primitive action exited with error!"))
        return response.get_result()


def main():
    ros2py_testing.run_ros1_test("test_fit_primitive", TestFitPrimitive)


if __name__ == "__main__":
    main()
