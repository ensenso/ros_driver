import json
import math

import tf


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
        position = [transform["Translation"][0] / 1000,
                    transform["Translation"][1] / 1000,
                    transform["Translation"][2] / 1000]
        axis = (transform["Rotation"]["Axis"][0], transform["Rotation"]["Axis"][1], transform["Rotation"]["Axis"][2])
        orientation = tf.transformations.quaternion_about_axis(transform["Rotation"]["Angle"], axis)

        return cls(position, orientation)

    @classmethod
    def from_message(cls, message):
        position = [message.position.x,
                    message.position.y,
                    message.position.z]
        orientation = [message.orientation.x,
                       message.orientation.y,
                       message.orientation.z,
                       message.orientation.w]

        return cls(position, orientation)

    def inverse(self):
        """
        Get the inverse of the pose.
        """
        translation = tf.transformations.translation_matrix(self.position)
        rotation = tf.transformations.quaternion_matrix(self.orientation)
        transform = tf.transformations.concatenate_matrices(translation, rotation)

        inverse_transform = tf.transformations.inverse_matrix(transform)
        translation = tf.transformations.translation_from_matrix(inverse_transform)
        rotation = tf.transformations.quaternion_from_matrix(inverse_transform)

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
        message.lower.x = self.lower[0]
        message.lower.y = self.lower[1]
        message.lower.z = self.lower[2]

        message.upper.x = self.upper[0]
        message.upper.y = self.upper[1]
        message.upper.z = self.upper[2]

    def is_empty(self):
        for l, u in zip(self.lower, self.upper):
            if l >= u:
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
