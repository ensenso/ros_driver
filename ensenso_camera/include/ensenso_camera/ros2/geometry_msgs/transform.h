#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <geometry_msgs/msg/transform.hpp>
#else
#include <geometry_msgs/Transform.h>
#endif

USING_MSG(geometry_msgs, Transform)
