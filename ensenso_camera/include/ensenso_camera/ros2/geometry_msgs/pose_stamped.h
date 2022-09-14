#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <geometry_msgs/msg/pose_stamped.hpp>
#else
#include <geometry_msgs/PoseStamped.h>
#endif

USING_MSG(geometry_msgs, PoseStamped)
