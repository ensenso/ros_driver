#pragma once

#include "nxLib.h"
#include "pose_utilities.h"

#include <tf2/LinearMath/Transform.h>
#include <boost/optional.hpp>
#include <string>
#include <vector>

/**
 * Check whether the NxLib has at least the given version.
 */
bool checkNxLibVersion(int major, int minor)
{
  int nxLibMajor = NxLibItem()[itmVersion][itmMajor].asInt();
  int nxLibMinor = NxLibItem()[itmVersion][itmMinor].asInt();
  return (nxLibMajor > major) || (nxLibMajor == major && nxLibMinor >= minor);
}

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

void setRenderParams(NxLibItem const& paramItem, RenderPointMapParams const* params)
{
  paramItem[itmUseOpenGL] = params->useOpenGl();
  if (params->pixelScale())
  {
    paramItem[itmPixelSize] = *params->pixelScale();
  }
  if (params->scaling())
  {
    paramItem[itmScaling] = *params->scaling();
  }
  if (params->withTexture())
  {
    paramItem[itmTexture] = *params->withTexture();
  }
  if (params->sizeWidth())
  {
    paramItem[itmSize][0] = *params->sizeWidth();
  }
  if (params->sizeHeight())
  {
    paramItem[itmSize][1] = *params->sizeHeight();
  }
  if (params->viewPose())
  {
    writePoseToNxLib(*params->viewPose(), paramItem[itmViewPose]);
  }
}

inline void rosWarnOnceAboutDeprSDKVersion(std::string const& notSupportedMsg, VersionInfo const& version)
{
  ROS_WARN_ONCE("This EnsensoSDK Version does not support: %s. Version used: %i.%i.%i", notSupportedMsg.c_str(),
                version.majorV, version.minorV, version.buildV);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr retrieveRenderedPointCloud(NxLibItem const& cmdResult,
                                                               VersionInfo const& nxLibVersion,
                                                               std::string const& frame)
{
  // Because the Ensenso SDK 2.2.x has its Rendered Point Maps in a global image node, we try to get the rendered point
  // cloud from this image node. Of course this is NOT Multi Threading friendly.
  if (nxLibVersion.majorV == 2 && nxLibVersion.minorV <= 2)
  {
    rosWarnOnceAboutDeprSDKVersion("Multi threaded Rendering of Point map", nxLibVersion);
    auto const& globalResults = NxLibItem()[itmImages];
    return pointCloudFromNxLib(globalResults[itmRenderPointMap], frame);
  }

  return pointCloudFromNxLib(cmdResult[itmImages][itmRenderPointMap], frame);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr retrieveTexturedPointCloud(NxLibItem const& cmdResult,
                                                                  VersionInfo const& nxLibVersion,
                                                                  std::string const& targetFrame)
{
  if (nxLibVersion.majorV == 2 && nxLibVersion.minorV <= 2)
  {
    rosWarnOnceAboutDeprSDKVersion("Multi threaded Rendering of Point map", nxLibVersion);
    auto const& globalResults = NxLibItem()[itmImages];
    return pointCloudTexturedFromNxLib(globalResults[itmRenderPointMapTexture], globalResults[itmRenderPointMap],
                                       targetFrame);
  }

  return pointCloudTexturedFromNxLib(cmdResult[itmImages][itmRenderPointMapTexture],
                                     cmdResult[itmImages][itmRenderPointMap], targetFrame);
}

NxLibItem retrieveResultPath(NxLibItem const& cmdResult, VersionInfo const& nxLibVersion)
{
  if (nxLibVersion.majorV == 2 && nxLibVersion.minorV <= 2)
  {
    return NxLibItem()[itmImages][itmRenderPointMap];
  }

  return cmdResult[itmImages][itmRenderPointMap];
}

sensor_msgs::ImagePtr retrieveRenderedDepthMap(NxLibItem const& cmdResult, VersionInfo const& nxLibVersion,
                                               std::string const& frame)
{
  sensor_msgs::ImagePtr renderedImage;
  if (nxLibVersion.majorV == 2 && nxLibVersion.minorV <= 2)
  {
    rosWarnOnceAboutDeprSDKVersion("Multi threaded Rendering of Point map", nxLibVersion);
    auto const& globalResults = NxLibItem()[itmImages];
    return depthImageFromNxLibNode(globalResults[itmRenderPointMap], frame);
  }

  return depthImageFromNxLibNode(cmdResult[itmImages][itmRenderPointMap], frame);
}

void setRenderParams(NxLibItem const& cmdParams, VersionInfo const& nxLibVersion, RenderPointMapParams const* params)
{
  // Some Parameters are set in the global parameters node in 2.2.x. That is why we have to split the params
  // correspondingly to the global and the command parameter node.
  if (nxLibVersion.majorV == 2 && nxLibVersion.minorV <= 2)
  {
    rosWarnOnceAboutDeprSDKVersion("Multi threaded Rendering of Point map", nxLibVersion);
    auto globalParams = NxLibItem()[itmParameters][itmRenderPointMap];
    setRenderParams(globalParams, params);
  }
  else
  {
    setRenderParams(cmdParams, params);
  }
  // Some parameters are both in the cmd Parameters and the global parameter node
  if (params->far())
  {
    cmdParams[itmFar] = *params->far();
  }
  if (params->near())
  {
    cmdParams[itmNear] = *params->near();
  }
}
