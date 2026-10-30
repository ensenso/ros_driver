#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <geometry_msgs/msg/point32.hpp>
#else
#include <geometry_msgs/Point32.h>
#endif

USING_MSG(geometry_msgs, Point32)
