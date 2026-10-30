#pragma once

#include "ensenso_camera/ros2/action_server.h"

#ifdef ROS2
#include "ensenso_camera_msgs/action/request_data_mono.hpp"
#else
#include "ensenso_camera_msgs/RequestDataMonoAction.h"
#endif

USING_ENSENSO_CAMERA_ACTION_WITH_SERVER(RequestDataMono)
