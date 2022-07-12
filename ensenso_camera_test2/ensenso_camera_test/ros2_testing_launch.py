# Output of ament_index_python methods for "ensenso_camera" in "~/ament_workspace":
# get_package_prefix 			-> ~/ament_workspace/install/ensenso_camera
# get_package_share_directory 	-> ~/ament_workspace/install/ensenso_camera/share/ensenso_camera

import os
import sys

from ament_index_python import get_package_prefix
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_testing.actions import ReadyToTest

from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

LAUNCH_PKG_PATH = "ensenso_camera"


def get_data_path(filename):
    return os.path.join(get_package_share_directory("ensenso_camera_test"), "data", filename)


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


def get_launch_description_for_unittest(launch_file_name, launch_arguments):
    launch_include = get_launch_include(LAUNCH_PKG_PATH, launch_file_name, launch_arguments)
    # Start tests right away - no need to wait for anything
    return LaunchDescription([launch_include, ReadyToTest()])


class EnsensoLaunchInclude:
    def __init__(self, name, args={}):
        self.name = name
        self.args = args

    def get_description(self):
        return get_launch_include("ensenso_camera", self.name, self.args)


class EnsensoScriptInclude:
    def __init__(self, name, args={}):
        self.name = name
        self.args = args

    def get_description(self):
        return get_dut_process("ensenso_camera", self.name, self.args)


def get_launch_description(*includes):
    return LaunchDescription([include.get_description() for include in includes] + [ReadyToTest()])
