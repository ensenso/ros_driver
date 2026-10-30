#pragma once

#include "ensenso_camera/ros2/action_server.h"

#ifdef ROS2
#include "ensenso_camera_msgs/action/locate_pattern_mono.hpp"
#else
#include "ensenso_camera_msgs/LocatePatternMonoAction.h"
#endif

USING_ENSENSO_CAMERA_ACTION_WITH_SERVER(LocatePatternMono)
