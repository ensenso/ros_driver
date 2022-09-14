#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <sensor_msgs/msg/image.hpp>
#else
#include <sensor_msgs/Image.h>
#endif

USING_MSG(sensor_msgs, Image)
