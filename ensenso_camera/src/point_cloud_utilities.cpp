#include "ensenso_camera/point_cloud_utilities.h"

#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>

double const NXLIB_TIMESTAMP_OFFSET = 11644473600;

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
  cloud->header.stamp = (timestamp - NXLIB_TIMESTAMP_OFFSET) * 1e6;
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

    if (roi != 0 && !roi->contains(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
    {
      cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return cloud;
}


//RGBD
boost::shared_ptr<sr::rgbd::Image> rgbdFromNxLib(NxLibItem const& node,
                                                 NxLibItem const& configPars,
                                                 std::string const& frame,
                                                 PointCloudROI const* roi)
{
  int width, height;
  double timestamp;
  std::vector<float> data;

  node.getBinaryDataInfo(&width, &height, 0, 0, 0, &timestamp);
  node.getBinaryData(data, 0);

  auto rgbd_image = boost::make_shared<sr::rgbd::Image>();

  double cx = configPars[2][0].asDouble();
  double cy = configPars[2][1].asDouble();
  double fx = configPars[0][0].asDouble();
  double fy = configPars[1][1].asDouble();

  //Move raw data to rgbd image
  rgbd_image->depth = cv::Mat( (unsigned int)height, (unsigned int)width, CV_32FC1, NAN);
  rgbd_image->timestamp = (timestamp - NXLIB_TIMESTAMP_OFFSET) * 1e3; //rgbd uses microseconds
  rgbd_image->frame_id = frame;
  rgbd_image->P.setOpticalTranslation(0, 0);
  rgbd_image->P.setOpticalCenter(cx, cy);
  rgbd_image->P.setFocalLengths(fx, fy);

  float px, py, pz;
  for(int i = 0; i < width * height; ++i)
  {
      px = data[i * 3];
      if (!std::isnan(px))
      {
          px /= 1000.;
          py = data[i * 3 + 1] / 1000;
          pz = data[i * 3 + 2] / 1000;
          sr::Vec2i pix = rgbd_image->P.project3Dto2D(sr::Vec3(px, -py, -pz));
          if (pix.x >= 0 && pix.y >= 0 && pix.x < rgbd_image->depth.cols && pix.y < rgbd_image->depth.rows)
          {
              rgbd_image->depth.at<float>(pix.y, pix.x) = pz;
              if (roi != 0 && !roi->contains(px, py, pz))
              {
                  rgbd_image->depth.at<float>(pix.y, pix.x) = std::numeric_limits<float>::quiet_NaN();
              }
          }
      }
  }

  return rgbd_image;
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
  cloud->header.stamp = (timestamp - NXLIB_TIMESTAMP_OFFSET) * 1e6;
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

    if (roi != 0 && !roi->contains(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
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
