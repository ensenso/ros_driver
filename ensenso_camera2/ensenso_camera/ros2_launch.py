# Output of ament_index_python methods for "ensenso_camera" in "~/ament_workspace":
# get_package_prefix 			-> ~/ament_workspace/install/ensenso_camera
# get_package_share_directory 	-> ~/ament_workspace/install/ensenso_camera/share/ensenso_camera

import os
import sys

from ament_index_python import get_package_prefix
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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


def get_dut_process(package_name, process_name, args={}):
    proc_path = os.path.join(
        get_package_prefix(package_name),
        f"lib/{package_name}",
        process_name,
    )

    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    cmd = [sys.executable, proc_path]
    if args:
        cmd.append("--ros-args")
        for arg_name, arg_value in args.items():
            cmd.extend(["-p", f"{arg_name}:={arg_value}"])

    return ExecuteProcess(cmd=cmd, env=proc_env, output="screen")


def get_launch_include(package_name, launch_file_name, launch_arguments={}):
    """
    https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/
    https://stackoverflow.com/questions/57696569/ros2-how-to-pass-arguments-from-one-launch-file-to-a-child-launch-file
    """
    launch_file_path = os.path.join(get_package_share_directory(package_name), f"launch/{launch_file_name}")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments=launch_arguments.items(),
    )


class LaunchFileInclude:
    def __init__(self, name, args={}):
        self.name = name
        self.args = args

    def get_description(self):
        return get_launch_include("ensenso_camera", self.name, self.args)

    def get_launch_description(self):
        return LaunchDescription([self.get_description()])


class ScriptInclude:
    def __init__(self, name, args={}):
        self.name = name
        self.args = args

    def get_description(self):
        return get_dut_process("ensenso_camera", self.name, self.args)
