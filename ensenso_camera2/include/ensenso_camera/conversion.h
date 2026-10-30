#pragma once

#include "ensenso_camera/ros2/namespace.h"

#include "ensenso_camera/ros2/geometry_msgs/point32.h"

#include "nxLib.h"

namespace ensenso_conversion
{
double nxLibToRosTimestamp(double const& timestamp, bool isFileCamera = false);

double nxLibToPclTimestamp(double const& timestamp, bool isFileCamera = false);

// Internally units are used with millimeters instead of meters, but ROS uses meters most often.
const int conversionFactor = 1000;

geometry_msgs::msg::Point32 toRosPoint(NxLibItem const& itemArray, bool convertUnits = true);

NxLibItem toEnsensoPoint(geometry_msgs::msg::Point32 const& point, bool convertUnits = true);

}  // namespace ensenso_conversion
