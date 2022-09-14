#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <sensor_msgs/msg/camera_info.hpp>
#else
#include <sensor_msgs/CameraInfo.h>
#endif

USING_MSG(sensor_msgs, CameraInfo)
