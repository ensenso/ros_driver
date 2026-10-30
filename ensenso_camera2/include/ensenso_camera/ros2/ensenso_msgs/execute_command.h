#pragma once

#include "ensenso_camera/ros2/action_server.h"

#ifdef ROS2
#include "ensenso_camera_msgs/action/execute_command.hpp"
#else
#include "ensenso_camera_msgs/ExecuteCommandAction.h"
#endif

USING_ENSENSO_CAMERA_ACTION_WITH_SERVER(ExecuteCommand)
