#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

#include "nxLib.h"

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

/**
 * Convert the given binary NxLib node to a PCL point cloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromNxLib(NxLibItem const& node, std::string const& frame,
                                                        PointCloudROI const* roi = 0);

/**
 * Create a PCL point cloud with normals from the given NxLib nodes.
 *
 * @param pointMapNode The NxLib node that contains the point map.
 * @param normalNode   The NxLib node that contains the normal information.
 */
pcl::PointCloud<pcl::PointNormal>::Ptr pointCloudWithNormalsFromNxLib(NxLibItem const& pointMapNode,
                                                                      NxLibItem const& normalNode,
                                                                      std::string const& frame,
                                                                      PointCloudROI const* roi = 0);
