#include "ensenso_camera/conversion.h"

namespace ensenso_conversion
{
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