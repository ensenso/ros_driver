#pragma once

#include "nxLib.h"
#include "pose_utilities.h"

#include <tf2/LinearMath/Transform.h>
#include <boost/optional.hpp>
#include <string>
#include <vector>

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

  virtual boost::optional<int> pixelScale() const
  {
    return {};
  }

  virtual boost::optional<double> scaling() const
  {
    return {};
  }

  virtual boost::optional<int> sizeWidth() const
  {
    return {};
  }

  virtual boost::optional<int> sizeHeight() const
  {
    return {};
  }

  virtual boost::optional<double> far() const
  {
    return {};
  }

  virtual boost::optional<double> near() const
  {
    return {};
  }

  virtual boost::optional<bool> withTexture() const
  {
    return {};
  }

  virtual boost::optional<tf2::Transform const&> viewPose() const
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
  tf2::Transform mViewPose;

public:
  RenderPointMapParamsTelecentric(bool useOpenGl, int pixelScale, double scaling, int sizeWidth, int sizeHeight,
                                  tf2::Transform const& viewPose)
    : RenderPointMapParams(useOpenGl)
    , mPixelScale(pixelScale)
    , mScaling(scaling)
    , mSizeWidth(sizeWidth)
    , mSizeHeight(sizeHeight)
    , mViewPose(viewPose){};

  boost::optional<int> pixelScale() const override
  {
    return mPixelScale;
  }

  boost::optional<double> scaling() const override
  {
    return mScaling;
  }

  boost::optional<int> sizeWidth() const override
  {
    return mSizeWidth;
  }

  boost::optional<int> sizeHeight() const override
  {
    return mSizeHeight;
  }

  boost::optional<tf2::Transform const&> viewPose() const override
  {
    if (!isValid(mViewPose))
    {
      return {};
    }
    return mViewPose;
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

  boost::optional<double> far() const override
  {
    return mFar;
  }

  boost::optional<double> near() const override
  {
    return mNear;
  }

  boost::optional<bool> withTexture() const override
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
  if (params->viewPose())
  {
    writePoseToNxLib(*params->viewPose(), cmdParams[itmViewPose]);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr retrieveRenderedPointCloud(NxLibItem const& cmdResult, std::string const& frame)
{
  return pointCloudFromNxLib(cmdResult[itmImages][itmRenderPointMap], frame);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr retrieveTexturedPointCloud(NxLibItem const& cmdResult,
                                                                  std::string const& targetFrame)
{
  return pointCloudTexturedFromNxLib(cmdResult[itmImages][itmRenderPointMapTexture],
                                     cmdResult[itmImages][itmRenderPointMap], targetFrame);
}

sensor_msgs::ImagePtr retrieveRenderedDepthMap(NxLibItem const& cmdResult, std::string const& frame, bool isFileCamera)
{
  sensor_msgs::ImagePtr renderedImage;
  return depthImageFromNxLibNode(cmdResult[itmImages][itmRenderPointMap], frame, isFileCamera);
}
