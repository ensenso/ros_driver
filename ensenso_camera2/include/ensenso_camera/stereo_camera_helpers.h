#pragma once

#include "ensenso_camera/ros2/core.h"

#include "ensenso_camera/conversion.h"
#include "ensenso_camera/point_cloud_utilities.h"
#include "ensenso_camera/pose_utilities.h"

#include <tf2/LinearMath/Transform.h>

#include <limits>
#include <string>
#include <vector>

#include "nxLib.h"

class RenderPointMapParams
{
protected:
  bool useOpenGL;

public:
  explicit RenderPointMapParams(bool _useOpenGl) : useOpenGL(_useOpenGl){};
  virtual bool useOpenGl() const
  {
    return useOpenGL;
  };

  virtual ensenso::std::optional<int> pixelScale() const
  {
    return {};
  }

  virtual ensenso::std::optional<double> scaling() const
  {
    return {};
  }

  virtual ensenso::std::optional<int> sizeWidth() const
  {
    return {};
  }

  virtual ensenso::std::optional<int> sizeHeight() const
  {
    return {};
  }

  virtual ensenso::std::optional<double> far() const
  {
    return {};
  }

  virtual ensenso::std::optional<double> near() const
  {
    return {};
  }

  virtual ensenso::std::optional<bool> withTexture() const
  {
    return {};
  }

  virtual ensenso::std::optional<tf2::Transform> transform() const
  {
    return {};
  }
};

class RenderPointMapParamsTelecentric : public RenderPointMapParams
{
private:
  int mPixelScale;
  double mScaling;
  int mSizeWidth;
  int mSizeHeight;
  tf2::Transform mTransform;

public:
  RenderPointMapParamsTelecentric(bool useOpenGl, int pixelScale, double scaling, int sizeWidth, int sizeHeight,
                                  tf2::Transform transform)
    : RenderPointMapParams(useOpenGl)
    , mPixelScale(pixelScale)
    , mScaling(scaling)
    , mSizeWidth(sizeWidth)
    , mSizeHeight(sizeHeight)
    , mTransform(std::move(transform))
  {
  }

  ensenso::std::optional<int> pixelScale() const override
  {
    return mPixelScale;
  }

  ensenso::std::optional<double> scaling() const override
  {
    return mScaling;
  }

  ensenso::std::optional<int> sizeWidth() const override
  {
    return mSizeWidth;
  }

  ensenso::std::optional<int> sizeHeight() const override
  {
    return mSizeHeight;
  }

  ensenso::std::optional<tf2::Transform> transform() const override
  {
    return mTransform;
  }
};

class RenderPointMapParamsProjection : public RenderPointMapParams
{
private:
  double mFar, mNear;
  bool mWithTexture;

public:
  RenderPointMapParamsProjection(bool useOpenGL, double far, double near, bool withTexture)
    : RenderPointMapParams(useOpenGL), mFar(far), mNear(near), mWithTexture(withTexture){};

  ensenso::std::optional<double> far() const override
  {
    return mFar;
  }

  ensenso::std::optional<double> near() const override
  {
    return mNear;
  }

  ensenso::std::optional<bool> withTexture() const override
  {
    return mWithTexture;
  }
};

void setRenderParams(NxLibItem const& cmdParams, RenderPointMapParams const* params)
{
  cmdParams[itmUseOpenGL] = params->useOpenGl();
  if (params->pixelScale())
  {
    cmdParams[itmPixelSize] = *params->pixelScale();
  }
  if (params->scaling())
  {
    cmdParams[itmScaling] = *params->scaling();
  }
  if (params->withTexture())
  {
    cmdParams[itmTexture] = *params->withTexture();
  }
  if (params->sizeWidth())
  {
    cmdParams[itmSize][0] = *params->sizeWidth();
  }
  if (params->sizeHeight())
  {
    cmdParams[itmSize][1] = *params->sizeHeight();
  }
  if (params->transform())
  {
    writeTransformToNxLib(*params->transform(), cmdParams[itmViewPose]);
  }

  // Some parameters are both in the command parameters and the global parameter node.
  if (params->far())
  {
    cmdParams[itmFar] = *params->far();
  }
  if (params->near())
  {
    cmdParams[itmNear] = *params->near();
  }
}

std::unique_ptr<ensenso::pcl::PointCloud> pointCloudFromNxLib(NxLibItem const& node, std::string const& frame,
                                                              bool isFileCamera = false,
                                                              PointCloudROI const* roi = nullptr)
{
  int width, height;
  double timestamp;
  std::vector<float> data;

  node.getBinaryDataInfo(&width, &height, 0, 0, 0, &timestamp);
  node.getBinaryData(data, 0);

  auto cloud = ensenso::std::make_unique<ensenso::pcl::PointCloud>();

  // PCL timestamp is in microseconds and Unix time.
  cloud->header.stamp = ensenso_conversion::nxLibToPclTimestamp(timestamp, isFileCamera);
  cloud->header.frame_id = frame;

  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  cloud->points.resize(width * height);

  for (int i = 0; i < width * height; i++)
  {
    // The NxLib point cloud is in millimeters, ROS needs everything in meters.
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

std::unique_ptr<ensenso::pcl::PointCloudNormals> pointCloudWithNormalsFromNxLib(NxLibItem const& pointMapNode,
                                                                                NxLibItem const& normalNode,
                                                                                std::string const& frame,
                                                                                bool isFileCamera = false,
                                                                                PointCloudROI const* roi = nullptr)
{
  int width, height;
  double timestamp;
  std::vector<float> pointData;
  std::vector<float> normalData;

  pointMapNode.getBinaryDataInfo(&width, &height, 0, 0, 0, &timestamp);
  pointMapNode.getBinaryData(pointData, 0);
  normalNode.getBinaryData(normalData, 0);

  auto cloud = ensenso::std::make_unique<ensenso::pcl::PointCloudNormals>();

  // PCL timestamp is in microseconds and Unix time.
  cloud->header.stamp = ensenso_conversion::nxLibToPclTimestamp(timestamp, isFileCamera);
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

std::unique_ptr<ensenso::pcl::PointCloudColored> pointCloudTexturedFromNxLib(NxLibItem const& imageNode,
                                                                             NxLibItem const& pointsNode,
                                                                             std::string const& frame,
                                                                             bool isFileCamera = false,
                                                                             PointCloudROI const* roi = nullptr)
{
  int width, height;
  double timestamp;
  std::vector<float> data;
  std::vector<unsigned char> imageData;

  pointsNode.getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
  pointsNode.getBinaryData(data, &timestamp);
  imageNode.getBinaryData(imageData, 0);

  auto cloud = ensenso::std::make_unique<ensenso::pcl::PointCloudColored>();

  // PCL timestamp is in microseconds and Unix time.
  cloud->header.stamp = ensenso_conversion::nxLibToPclTimestamp(timestamp, isFileCamera);
  cloud->header.frame_id = frame;

  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  cloud->points.resize(width * height);

  for (int i = 0; i < width * height; i++)
  {
    // The NxLib point cloud is in millimeters, ROS needs everything in meters.
    cloud->points[i].x = data[3 * i] / 1000.0f;
    cloud->points[i].y = data[3 * i + 1] / 1000.0f;
    cloud->points[i].z = data[3 * i + 2] / 1000.0f;

    cloud->points[i].r = imageData[4 * i];
    cloud->points[i].g = imageData[4 * i + 1];
    cloud->points[i].b = imageData[4 * i + 2];
    cloud->points[i].a = imageData[4 * i + 3];

    if (roi != nullptr && !roi->contains(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
    {
      cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
      cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return cloud;
}

std::unique_ptr<ensenso::pcl::PointCloud> retrieveRenderedPointCloud(NxLibItem const& cmdResult,
                                                                     std::string const& frame,
                                                                     bool isFileCamera = false)
{
  return pointCloudFromNxLib(cmdResult[itmImages][itmRenderPointMap], frame, isFileCamera);
}

std::unique_ptr<ensenso::pcl::PointCloudColored> retrieveTexturedPointCloud(NxLibItem const& cmdResult,
                                                                            std::string const& targetFrame,
                                                                            bool isFileCamera = false)
{
  return pointCloudTexturedFromNxLib(cmdResult[itmImages][itmRenderPointMapTexture],
                                     cmdResult[itmImages][itmRenderPointMap], targetFrame, isFileCamera);
}

sensor_msgs::msg::ImagePtr retrieveRenderedDepthMap(NxLibItem const& cmdResult, std::string const& frame,
                                                    bool isFileCamera)
{
  sensor_msgs::msg::ImagePtr renderedImage;
  return depthImageFromNxLibNode(cmdResult[itmImages][itmRenderPointMap], frame, isFileCamera);
}
