EMPTY_STRING = "None"


def fix_empty_string(value):
    """
    Fixes problem of buggy empty default values of launch arguments in ROS2-Foxy.
    See the following issue and the issues linked below the closure: https://github.com/introlab/rtabmap_ros/issues/725
    """
    if value == EMPTY_STRING:
        return ""
    return value


def get_camera_frame(serial, camera_frame):
    """
    Return the default camera frame name as it is created in the C++ implementation if no camera_frame is given.
    Otherwise return the given camera frame.
    """
    if camera_frame == EMPTY_STRING:
        return f"optical_frame_{serial}"
    return camera_frame
