#pragma once

#include <geometry_msgs/Point32.h>

#include "nxLib.h"

namespace ensenso_conversion
{
// Internally units are used with millimeters instead of meters, but ROS uses meters most often.
const int conversionFactor = 1000;

geometry_msgs::Point32 toRosPoint(NxLibItem const& itemArray, bool convertUnits = true);

NxLibItem toEnsensoPoint(geometry_msgs::Point32 const& point, bool convertUnits = true);

}  // namespace ensenso_conversion
