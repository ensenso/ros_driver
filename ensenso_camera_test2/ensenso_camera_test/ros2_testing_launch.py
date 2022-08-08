# Output of ament_index_python methods for "ensenso_camera" in "~/ament_workspace":
# get_package_prefix 			-> ~/ament_workspace/install/ensenso_camera
# get_package_share_directory 	-> ~/ament_workspace/install/ensenso_camera/share/ensenso_camera

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction

from launch_testing.actions import ReadyToTest

from ensenso_camera.ros2_launch import LaunchFileInclude as _EnsensoLaunchInclude
from ensenso_camera.ros2_launch import ScriptInclude as _EnsensoScriptInclude


LAUNCH_DELAY_PERIOD_IN_S = 5.0


EnsensoLaunchInclude = _EnsensoLaunchInclude
EnsensoScriptInclude = _EnsensoScriptInclude


def get_data_path(filename):
    return os.path.join(get_package_share_directory("ensenso_camera_test"), "data", filename)


def generate_test_description(*includes):
    descriptions = [include.get_description() for include in includes]
    descriptions.append(TimerAction(period=LAUNCH_DELAY_PERIOD_IN_S, actions=[ReadyToTest()]))
    return LaunchDescription(descriptions)
