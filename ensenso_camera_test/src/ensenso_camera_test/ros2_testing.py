from ensenso_camera.ros2 import import_from_module
from ensenso_camera.ros2 import is_ros2

TEST_PACKAGE_NAME = "ensenso_camera_test"
TEST_TIMEOUT = 20


# ----------------------------------------------------------------------------------------------------------------------
# ROS2
# ----------------------------------------------------------------------------------------------------------------------
if is_ros2():

    def import_point_cloud2():
        return import_from_module("ensenso_camera_test", "ros2_point_cloud2")

    def import_tf_transformation_function(function_name):
        # requires: sudo apt-get install ros-galactic-tf-transformations
        # requires: sudo pip3 install transforms3d
        # In this order !!!
        # See (1) https://github.com/ros/geometry_tutorials/issues/67
        # See (2) https://github.com/DLu/tf_transformations
        return import_from_module("tf_transformations", function_name)

    def get_test_data_path(path):
        import os
        from ament_index_python import get_package_share_directory

        return os.path.join(get_package_share_directory("ensenso_camera_test"), "data", path)

    def create_tf_broadcaster(node):
        from tf2_ros import TransformBroadcaster

        return TransformBroadcaster(node)

    def create_tf_transform(pose, timestamp, child_frame, parent_frame):
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = pose.position[0]
        t.transform.translation.y = pose.position[1]
        t.transform.translation.z = pose.position[2]

        t.transform.rotation.x = pose.orientation[0]
        t.transform.rotation.y = pose.orientation[1]
        t.transform.rotation.z = pose.orientation[2]
        t.transform.rotation.w = pose.orientation[3]

        return t

    def send_tf_transform(broadcaster, pose, timestamp, child_frame, parent_frame):
        broadcaster.sendTransform(create_tf_transform(pose, timestamp, child_frame, parent_frame))

    def to_sec(t):
        from builtin_interfaces.msg import Time as MsgTime
        from rclpy.time import Time as RclTime

        if isinstance(t, MsgTime):
            return t.sec
        elif isinstance(t, RclTime):
            return t.seconds_nanoseconds()[0]
        raise TypeError("Cannot convert type {} to seconds".format(type(t)))

    def feedback_callback(func):
        def wrapper(feedback):
            func(feedback.feedback)

        return wrapper


# ----------------------------------------------------------------------------------------------------------------------
# ROS1
# ----------------------------------------------------------------------------------------------------------------------
else:

    def import_point_cloud2():
        return import_from_module("sensor_msgs", "point_cloud2")

    def import_tf_transformation_function(function_name):
        return import_from_module("tf.transformations", function_name)

    def get_test_data_path(path):
        return "../../data/" + path.lstrip("/")

    def create_tf_broadcaster(_):
        from tf import TransformBroadcaster

        return TransformBroadcaster()

    def send_tf_transform(broadcaster, pose, timestamp, child_frame, parent_frame):
        broadcaster.sendTransform(pose.position, pose.orientation, timestamp, child_frame, parent_frame)

    def to_sec(t):
        try:
            return t.to_sec()
        except TypeError:
            raise TypeError("Cannot convert type {} to seconds".format(type(t)))

    def feedback_callback(func):
        def wrapper(feedback):
            func(feedback)

        return wrapper


def run_ros1_test(test_name, test):
    import rospy
    import rostest

    try:
        rostest.rosrun(TEST_PACKAGE_NAME, test_name, test)
    except rospy.ROSInterruptException:
        pass
