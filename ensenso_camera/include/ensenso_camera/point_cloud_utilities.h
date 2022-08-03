#pragma once

#include <pcl_ros/point_cloud.h>

#include <string>

#include "nxLib.h"

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

/**
 * Convert the given binary NxLib node to a PCL point cloud.
 */
ensenso::pcl::PointCloud::Ptr pointCloudFromNxLib(NxLibItem const& node, std::string const& frame,
                                                  bool isFileCamera = false, PointCloudROI const* roi = nullptr);

/**
 * Create a PCL point cloud with normals from the given NxLib nodes.
 */
ensenso::pcl::PointCloudNormals::Ptr pointCloudWithNormalsFromNxLib(NxLibItem const& pointMapNode,
                                                                    NxLibItem const& normalNode,
                                                                    std::string const& frame, bool isFileCamera = false,
                                                                    PointCloudROI const* roi = nullptr);

/**
 * Create a PCL point cloud with texture from the given NxLib nodes.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTexturedFromNxLib(NxLibItem const& imageNode,
                                                                   NxLibItem const& pointsNode,
                                                                   std::string const& frame, bool isFileCamera = false,
                                                                   PointCloudROI const* roi = nullptr);
