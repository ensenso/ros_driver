#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#else
#include <diagnostic_msgs/DiagnosticArray.h>
#endif

USING_MSG(diagnostic_msgs, DiagnosticArray)
