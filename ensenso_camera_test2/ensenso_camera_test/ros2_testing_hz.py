import threading
import time
import unittest

import ensenso_camera.ros2 as ros2py


class HzTestParameters:
    def __init__(self, rate_in_hz, tolerance_in_hz, duration_in_s):
        self.rate_in_hz = rate_in_hz
        self.tolerance_in_hz = tolerance_in_hz
        self.duration_in_s = duration_in_s


class HzTestSampler:
    def __init__(self, test_name, msg_type, topic, duration_in_s):
        self.test_name = test_name
        self.msg_type = msg_type
        self.topic = topic
        self.duration_in_s = duration_in_s

        self.lock = threading.Lock()
        self.t0 = -1
        self.rates = []

        self.node = ros2py.create_node(self.test_name)
        self.subscription = self.node.create_subscription(self.msg_type, self.topic, self.subscription_callback, 10)
        ros2py.spin(self.node)

    def run(self):
        loop_rate = self.node.create_rate(0.1)
        loop_duration = 0
        loop_t0 = time.time()
        while ros2py.ok() and loop_duration < self.duration_in_s:
            loop_rate.sleep()
            loop_duration = time.time() - loop_t0
        ros2py.shutdown(self.node)
        return self.rates

    def subscription_callback(self, msg):
        with self.lock:
            if self.t0 > 0:
                rate = time.time() - self.t0
                self.rates.append(rate)
            self.t0 = time.time()


def snake_to_upper_camel_case(s):
    return "".join(part.title() for part in s.split("_"))


def test_func(self):
    measured_rates = HzTestSampler(self.test_name, self.msg_type, self.topic, self.params.duration_in_s).run()
    self.assertTrue(len(measured_rates) > 0, msg=f"No messages received on topic {self.topic}")

    # TODO Find out why some of the GitHub action runs fail with very large rate deviations and reenable checking for
    # the actual average rate.
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
