#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include "ensenso_camera_msgs/msg/parameter.hpp"
#else
#include "ensenso_camera_msgs/Parameter.h"
#endif

USING_ENSENSO_CAMERA_MSG(Parameter)
