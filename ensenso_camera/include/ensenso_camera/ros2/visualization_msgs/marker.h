#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <visualization_msgs/msg/marker.hpp>
#else
#include <visualization_msgs/Marker.h>
#endif

USING_MSG(visualization_msgs, Marker)
