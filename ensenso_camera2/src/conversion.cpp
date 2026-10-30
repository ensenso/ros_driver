#include "ensenso_camera/conversion.h"

#include "ensenso_camera/ros2/logging.h"

#include <chrono>
#include <cmath>

namespace ensenso_conversion
{
namespace
{
double const NXLIB_TIMESTAMP_OFFSET = 11644473600;

double timeNowAsSeconds()
{
  // TODO Check if this also works with Windows and Mac in ROS2
  auto t = std::chrono::system_clock::now();
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  return (double)nanoseconds / 1e09;
}

double fixTimestamp(double const& timestamp, bool isFileCamera = false)
{
  if (std::isnan(timestamp))
  {
    // Handle missing timestamps.
    ENSENSO_WARN("NxLib timestamp is \'nan\', using current ROS time instead.");
    return timeNowAsSeconds();
  }
  else if (isFileCamera)
  {
    // File camera timestamps are too old for ROS, use the current time instead.
    return timeNowAsSeconds();
  }

  return timestamp - NXLIB_TIMESTAMP_OFFSET;
}
}  // namespace

double nxLibToRosTimestamp(double const& timestamp, bool isFileCamera)
{
  return fixTimestamp(timestamp, isFileCamera);
}

double nxLibToPclTimestamp(double const& timestamp, bool isFileCamera)
{
  // PCL timestamp is in microseconds and Unix time.
  return nxLibToRosTimestamp(timestamp, isFileCamera) * 1e6;
}

geometry_msgs::msg::Point32 toRosPoint(NxLibItem const& itemArray, bool convertUnits)
{
  geometry_msgs::msg::Point32 point;
  if (convertUnits)
  {
    point.x = itemArray[0].asDouble() / conversionFactor;
    point.y = itemArray[1].asDouble() / conversionFactor;
    point.z = itemArray[2].asDouble() / conversionFactor;
  }
  else
  {
    point.x = itemArray[0].asDouble();
    point.y = itemArray[1].asDouble();
    point.z = itemArray[2].asDouble();
  }

  return point;
}

NxLibItem toEnsensoPoint(geometry_msgs::msg::Point32 const& point, bool convertUnits)
{
  NxLibItem itemPoint;
  if (convertUnits)
  {
    itemPoint[0] = point.x * conversionFactor;
    itemPoint[1] = point.y * conversionFactor;
    itemPoint[2] = point.z * conversionFactor;
  }
  else
  {
    itemPoint[0] = point.x;
    itemPoint[1] = point.y;
    itemPoint[2] = point.z;
  }

  return itemPoint;
}

}  // namespace ensenso_conversion
