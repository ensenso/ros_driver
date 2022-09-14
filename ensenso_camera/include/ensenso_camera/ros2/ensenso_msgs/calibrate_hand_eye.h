#pragma once

#include "ensenso_camera/ros2/action_server.h"

#ifdef ROS2
#include "ensenso_camera_msgs/action/calibrate_hand_eye.hpp"
#else
#include "ensenso_camera_msgs/CalibrateHandEyeAction.h"
#endif

USING_ENSENSO_CAMERA_ACTION_WITH_SERVER(CalibrateHandEye)
