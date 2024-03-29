#pragma once

#include "ensenso_camera/ros2/pcl.h"

namespace ensenso
{
namespace pcl
{
using PointCloud = ::pcl::PointCloud<::pcl::PointXYZ>;
using PointCloudNormals = ::pcl::PointCloud<::pcl::PointNormal>;
using PointCloudColored = ::pcl::PointCloud<::pcl::PointXYZRGB>;
}  // namespace pcl
}  // namespace ensenso

struct PointCloudROI
{
  float minX = 0;
  float minY = 0;
  float minZ = 0;
  float maxX = 0;
  float maxY = 0;
  float maxZ = 0;

  bool contains(float x, float y, float z) const
  {
    return (x >= minX && x <= maxX && y >= minY && y <= maxY && z >= minZ && z <= maxZ);
  }

  bool isEmpty() const
  {
    return (minX >= maxX || minY >= maxY || minZ >= maxZ);
  }
};
