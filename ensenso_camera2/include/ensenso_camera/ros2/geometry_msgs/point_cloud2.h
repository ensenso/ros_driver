#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <geometry_msgs/msg/point_cloud2.hpp>
#else
#include <geometry_msgs/PointCloud2.h>
#endif

USING_MSG(geometry_msgs, PointCloud2)
