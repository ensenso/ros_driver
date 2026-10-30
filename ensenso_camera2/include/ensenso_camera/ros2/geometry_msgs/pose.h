#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <geometry_msgs/msg/pose.hpp>
#else
#include <geometry_msgs/Pose.h>
#endif

USING_MSG(geometry_msgs, Pose)
