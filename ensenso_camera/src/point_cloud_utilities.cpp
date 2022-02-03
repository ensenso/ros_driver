#include "ensenso_camera/point_cloud_utilities.h"

#include "ensenso_camera/conversion.h"

#include <pcl/point_types.h>

#include <limits>
#include <string>
#include <vector>
#include <ros/ros.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromNxLib(NxLibItem const& node, std::string const& frame,
                                                        PointCloudROI const* roi)
{
  int width, height;
  double timestamp;
  std::vector<float> data;

  node.getBinaryDataInfo(&width, &height, 0, 0, 0, &timestamp);
  node.getBinaryData(data, 0);

  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // PCL timestamp is in microseconds and Unix time.
  cloud->header.stamp = ensenso_conversion::nxLibToPclTimestamp(timestamp);
  cloud->header.frame_id = frame;

  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  cloud->points.resize(width * height);
  for (int i = 0; i < width * height; i++)
  {
    cloud->points[i].x = data[3 * i] / 1000.0f;
    cloud->points[i].y = data[3 * i + 1] / 1000.0f;
    cloud->points[i].z = data[3 * i + 2] / 1000.0f;

    if (roi != nullptr && !roi->contains(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
    {
      cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr pointCloudWithNormalsFromNxLib(NxLibItem const& pointMapNode,
                                                                      NxLibItem const& normalNode,
                                                                      std::string const& frame,
                                                                      PointCloudROI const* roi)
{
  int width, height;
  double timestamp;
  std::vector<float> pointData;
  std::vector<float> normalData;

  pointMapNode.getBinaryDataInfo(&width, &height, 0, 0, 0, &timestamp);
  pointMapNode.getBinaryData(pointData, 0);
  normalNode.getBinaryData(normalData, 0);

  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  // PCL timestamp is in microseconds and Unix time.
  cloud->header.stamp = ensenso_conversion::nxLibToPclTimestamp(timestamp);
  cloud->header.frame_id = frame;

  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  cloud->points.resize(width * height);
  for (int i = 0; i < width * height; i++)
  {
    // The NxLib point cloud is in millimeters, ROS needs everything in meters.
    cloud->points[i].x = pointData[3 * i] / 1000.0f;
    cloud->points[i].y = pointData[3 * i + 1] / 1000.0f;
    cloud->points[i].z = pointData[3 * i + 2] / 1000.0f;

    if (roi != nullptr && !roi->contains(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
    {
      cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    cloud->points[i].normal_x = normalData[3 * i];
    cloud->points[i].normal_y = normalData[3 * i + 1];
    cloud->points[i].normal_z = normalData[3 * i + 2];
  }

  return cloud;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTexturedFromNxLib(NxLibItem const& imageNode,
                                                                   NxLibItem const& pointsNode,
                                                                   std::string const& frame, PointCloudROI const* roi)
{
  double timestamp;

  int width, height;
  std::vector<float> data;
  std::vector<unsigned char> imageData;

  pointsNode.getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
  pointsNode.getBinaryData(data, &timestamp);
  imageNode.getBinaryData(imageData, 0);

  auto cloud_colored = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // PCL timestamp is in microseconds and Unix time.
  cloud_colored->header.stamp = ensenso_conversion::nxLibToPclTimestamp(timestamp);
  cloud_colored->header.frame_id = frame;

  cloud_colored->width = width;
  cloud_colored->height = height;
  cloud_colored->is_dense = false;
  cloud_colored->points.resize(width * height);

  for (int i = 0; i < width * height; i++)
  {
    cloud_colored->points[i].x = data[3 * i] / 1000.0f;
    cloud_colored->points[i].y = data[3 * i + 1] / 1000.0f;
    cloud_colored->points[i].z = data[3 * i + 2] / 1000.0f;

    cloud_colored->points[i].r = imageData[4 * i];
    cloud_colored->points[i].g = imageData[4 * i + 1];
    cloud_colored->points[i].b = imageData[4 * i + 2];
    cloud_colored->points[i].a = imageData[4 * i + 3];

    if (roi != nullptr &&
        !roi->contains(cloud_colored->points[i].x, cloud_colored->points[i].y, cloud_colored->points[i].z))
    {
      cloud_colored->points[i].x = std::numeric_limits<float>::quiet_NaN();
      cloud_colored->points[i].y = std::numeric_limits<float>::quiet_NaN();
      cloud_colored->points[i].z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return cloud_colored;
}
