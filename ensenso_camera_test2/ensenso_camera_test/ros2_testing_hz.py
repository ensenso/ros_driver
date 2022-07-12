import time
import unittest

import rclpy

import ensenso_camera.ros2 as ros2py


class HzTestError(Exception):
    pass


class HzTestParameters:
    def __init__(self, rate_in_hz, tolerance_in_hz, duration_in_s):
        self.rate_in_hz = rate_in_hz
        self.tolerance_in_hz = tolerance_in_hz
        self.duration_in_s = duration_in_s


class HzTestSampler:
    def __init__(self, test_name, msg_type, topic, params):
        self.test_name = test_name
        self.msg_type = msg_type
        self.topic = topic
        self.params = params

        self.node = ros2py.create_node(self.test_name)
        self.subscription = self.node.create_subscription(self.msg_type, self.topic, self.subscription_callback, 10)
        ros2py.spin(self.node)

    def run(self):
        self.measured_rates = []
        self.last_start_time = time.time()

        rate = self.node.create_rate(0.1)
        loop_duration = 0
        loop_start_time = time.time()
        while rclpy.ok() and loop_duration < self.params.duration_in_s:
            rate.sleep()
            loop_duration = time.time() - loop_start_time

        ros2py.shutdown(self.node)

        # TODO Reenable discarding in case average can be calculated correctly.
        # Discard the first measurement, because it is longer due to the setup
        # return self.measured_rates[1:]

        return self.measured_rates

    def subscription_callback(self, msg):
        current_rate = time.time() - self.last_start_time
        self.measured_rates.append(current_rate)
        self.last_start_time = time.time()


def snake_to_upper_camel_case(s):
    return "".join(part.title() for part in s.split("_"))


def test_func(self):
    measured_rates = HzTestSampler(self.test_name, self.msg_type, self.topic, self.params).run()
    self.assertTrue(len(measured_rates) > 0, msg=f"No messages received on topic {self.topic}")
    # TODO Fix this test or keep it like this and simply check if something is published at all.
    # Problem: The values measured by HzTestSampler are much larger than the values of 'ros2 topic hz <topic_name>'.
    # print(measured_rates)
    # average_rate = sum(measured_rates) / len(measured_rates)
    # self.assertAlmostEqual(average_rate, self.params.rate_in_hz, delta=self.params.tolerance_in_hz)


def HzTest(test_name, msg_type, topic, params):
    """Create a Hz test class based on unittest.TestCase."""
    assert test_name.startswith("test_")
    class_name = snake_to_upper_camel_case(test_name)
    class_bases = (unittest.TestCase,)
    class_dict = {
        "test_name": test_name,
        "msg_type": msg_type,
        "topic": topic,
        "params": params,
        test_name: test_func,
    }
    return type(class_name, class_bases, class_dict)
