#pragma once

#include <pcl_ros/point_cloud.h>

#include <string>

#include "nxLib.h"

namespace ensenso
{
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudNormals = pcl::PointCloud<pcl::PointNormal>;
using PointCloudColored = pcl::PointCloud<pcl::PointXYZRGB>;
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

/**
 * Convert the given binary NxLib node to a PCL point cloud.
 */
ensenso::PointCloud::Ptr pointCloudFromNxLib(NxLibItem const& node, std::string const& frame,
                                             PointCloudROI const* roi = nullptr);

/**
 * Create a PCL point cloud with normals from the given NxLib nodes.
 */
ensenso::PointCloudNormals::Ptr pointCloudWithNormalsFromNxLib(NxLibItem const& pointMapNode,
                                                               NxLibItem const& normalNode, std::string const& frame,
                                                               PointCloudROI const* roi = nullptr);

/**
 * Create a PCL point cloud with texture from the given NxLib nodes.
 */
ensenso::PointCloudColored::Ptr pointCloudTexturedFromNxLib(NxLibItem const& imageNode, NxLibItem const& pointsNode,
                                                            std::string const& frame,
                                                            PointCloudROI const* roi = nullptr);
