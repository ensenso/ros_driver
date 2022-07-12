#include "ensenso_camera/conversion.h"

#include "ros/ros.h"

#include <cmath>

namespace ensenso_conversion
{
namespace {
double const NXLIB_TIMESTAMP_OFFSET = 11644473600;

double fixTimestamp(double const& timestamp, bool isFileCamera = false)
{
  if (std::isnan(timestamp))
  {
    // Due to a bug in the NxLib of SDK 3.2.489, S-series file cameras have missing timestamps.
    // TODO: Remove this check as soon as we only support SDK versions that include the bug fix (SDK-2860).
    ROS_WARN("NxLib timestamp is \'nan\', using current ROS time instead.");
    return ros::Time::now().toSec();
  }
  else if (isFileCamera)
  {
    // File camera timestamps are too old for ROS, use the current time instead.
    return ros::Time::now().toSec();
  }
  return timestamp - NXLIB_TIMESTAMP_OFFSET;
}
}

double nxLibToRosTimestamp(double const& timestamp, bool isFileCamera)
{
  return fixTimestamp(timestamp, isFileCamera);
}

double nxLibToPclTimestamp(double const& timestamp)
{
  // PCL timestamp is in microseconds and Unix time.
  // No need for specifying the file camera flag, because point clouds are re-calculated from the saved images and thus
  // have a fresh timestamp in NxLib.
  return nxLibToRosTimestamp(timestamp) * 1e6;
}

geometry_msgs::Point32 toRosPoint(NxLibItem const& itemArray, bool convertUnits)
{
  geometry_msgs::Point32 point;
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

NxLibItem toEnsensoPoint(geometry_msgs::Point32 const& point, bool convertUnits)
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