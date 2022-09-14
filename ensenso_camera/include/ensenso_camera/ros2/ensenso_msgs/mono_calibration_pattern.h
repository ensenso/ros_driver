#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include "ensenso_camera_msgs/msg/mono_calibration_pattern.hpp"
#else
#include "ensenso_camera_msgs/MonoCalibrationPattern.h"
#endif

USING_ENSENSO_CAMERA_MSG(MonoCalibrationPattern)
