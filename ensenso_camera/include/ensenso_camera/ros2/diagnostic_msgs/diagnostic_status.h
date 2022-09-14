#pragma once

#include "ensenso_camera/ros2/namespace.h"

#ifdef ROS2
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#else
#include <diagnostic_msgs/DiagnosticStatus.h>
#endif

USING_MSG(diagnostic_msgs, DiagnosticStatus)
