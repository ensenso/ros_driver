import json
import math

from ensenso_camera_test.ros2_testing import import_tf_transformation_function

quaternion_about_axis = import_tf_transformation_function("quaternion_about_axis")
translation_matrix = import_tf_transformation_function("translation_matrix")
quaternion_matrix = import_tf_transformation_function("quaternion_matrix")
concatenate_matrices = import_tf_transformation_function("concatenate_matrices")
inverse_matrix = import_tf_transformation_function("inverse_matrix")
translation_from_matrix = import_tf_transformation_function("translation_from_matrix")
quaternion_from_matrix = import_tf_transformation_function("quaternion_from_matrix")


class Pose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation

    @classmethod
    def from_json(cls, path):
        with open(path) as f:
            transform = json.load(f)

        # The NxLib transformation in the JSON file is in millimeters. We
        # represent it in meters like everything in ROS.
        position = [
            transform["Translation"][0] / 1000,
            transform["Translation"][1] / 1000,
            transform["Translation"][2] / 1000,
        ]
        axis = (transform["Rotation"]["Axis"][0], transform["Rotation"]["Axis"][1], transform["Rotation"]["Axis"][2])
        orientation = quaternion_about_axis(transform["Rotation"]["Angle"], axis)

        return cls(position, orientation)

    @classmethod
    def from_message(cls, message):
        position = [message.position.x, message.position.y, message.position.z]
        orientation = [message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w]

        return cls(position, orientation)

    def inverse(self):
        """
        Get the inverse of the pose.
        """
        translation = translation_matrix(self.position)
        rotation = quaternion_matrix(self.orientation)
        transform = concatenate_matrices(translation, rotation)

        inverse_transform = inverse_matrix(transform)
        translation = translation_from_matrix(inverse_transform)
        rotation = quaternion_from_matrix(inverse_transform)

        return Pose(translation, rotation)

    def equals(self, other, tolerance=0.002):
        """
        Check whether this pose is equal to another one. The poses are
        considered equal, when all of the entries are within the given tolerance
        of each other.
        """
        for a, b in zip(self.position, other.position):
            if abs(a - b) > tolerance:
                return False
        for a, b in zip(self.orientation, other.orientation):
            if abs(a - b) > tolerance:
                return False
        return True


class ImagePoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @classmethod
    def from_message(cls, message):
        return cls(message.x, message.y)

    def distance_to(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def equals(self, other, tolerance=1):
        """
        Check whether the points are the same, up to the given distance.
        """
        return self.distance_to(other) < tolerance


class Point:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def dot_prod(self, other_point):
        return self.x * other_point.x + self.y * other_point.y + self.z * other_point.z


class RegionOfInterest:
    def __init__(self, lower, upper):
        self.lower = lower
        self.upper = upper

    @classmethod
    def from_message(cls, message):
        lower = [message.lower.x, message.lower.y, message.lower.z]
        upper = [message.upper.x, message.upper.y, message.upper.z]

        return cls(lower, upper)

    def write_to_message(self, message):
        message.lower.x = float(self.lower[0])
        message.lower.y = float(self.lower[1])
        message.lower.z = float(self.lower[2])

        message.upper.x = float(self.upper[0])
        message.upper.y = float(self.upper[1])
        message.upper.z = float(self.upper[2])

    def is_empty(self):
        for lower, upper in zip(self.lower, self.upper):
            if lower >= upper:
                return True
        return False

    def equals(self, other, tolerance=0):
        for a, b in zip(self.lower, other.lower):
            if abs(a - b) > tolerance:
                return False
        for a, b in zip(self.upper, other.upper):
            if abs(a - b) > tolerance:
                return False
        return True
