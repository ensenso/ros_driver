#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <visualization_msgs/msg/marker_array.hpp>
#else
#include <visualization_msgs/MarkerArray.h>
#endif

USING_MSG(visualization_msgs, MarkerArray)
