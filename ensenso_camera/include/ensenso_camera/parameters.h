#pragma once

#include "ensenso_camera_msgs/Parameter.h"

#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "nxLib.h"

/**
 * Possible types of parameters that are mapped to nodes of the NxLib.
 */
enum class ParameterType
{
  Bool,
  Number,  // Used for integers and floats.
  String
};

struct ParameterMapping
{
  ParameterType type;

  // The path to the corresponding NxLib node, relative to the camera's parameter node.
  std::vector<std::string> path;

  /**
   * Get the NxLib node that is referenced by this parameter.
   */
  NxLibItem node(NxLibItem const& cameraNode)
  {
    auto node = cameraNode[itmParameters];
    for (auto const& pathElement : path)
    {
      node = node[pathElement];
    }

    return node;
  }
};

/**
 * Map ROS parameters (as defined in ensenso_camera_msgs/Parameter) to NxLib
 * nodes.
 * Note that there are other parameters that cannot be mapped directly to an
 * NxLib node. These are handled in the parameter reading and writing methods
 * of the camera class.
 */
std::map<std::string, ParameterMapping> const parameterInformation{ //NOLINT
  // Capture parameters.
  { ensenso_camera_msgs::Parameter::AUTO_EXPOSURE, { ParameterType::Bool, { itmCapture, itmAutoExposure } } },
  { ensenso_camera_msgs::Parameter::AUTO_GAIN, { ParameterType::Bool, { itmCapture, itmAutoGain } } },
  { ensenso_camera_msgs::Parameter::BINNING, { ParameterType::Number, { itmCapture, itmBinning } } },
  { ensenso_camera_msgs::Parameter::EXPOSURE, { ParameterType::Number, { itmCapture, itmExposure } } },
  { ensenso_camera_msgs::Parameter::FRONT_LIGHT, { ParameterType::Bool, { itmCapture, itmFrontLight } } },
  { ensenso_camera_msgs::Parameter::GAIN, { ParameterType::Number, { itmCapture, itmGain } } },
  { ensenso_camera_msgs::Parameter::GAIN_BOOST, { ParameterType::Bool, { itmCapture, itmGainBoost } } },
  { ensenso_camera_msgs::Parameter::HARDWARE_GAMMA, { ParameterType::Bool, { itmCapture, itmHardwareGamma } } },
  { ensenso_camera_msgs::Parameter::MAX_GAIN, { ParameterType::Number, { itmCapture, itmMaxGain } } },
  { ensenso_camera_msgs::Parameter::PIXEL_CLOCK, { ParameterType::Number, { itmCapture, itmPixelClock } } },
  { ensenso_camera_msgs::Parameter::PROJECTOR, { ParameterType::Bool, { itmCapture, itmProjector } } },
  { ensenso_camera_msgs::Parameter::TARGET_BRIGHTNESS, { ParameterType::Number, { itmCapture, itmTargetBrightness } } },
  { ensenso_camera_msgs::Parameter::TRIGGER_DELAY, { ParameterType::Number, { itmCapture, itmTriggerDelay } } },
  { ensenso_camera_msgs::Parameter::TRIGGER_MODE, { ParameterType::String, { itmCapture, itmTriggerMode } } },

  // Matching parameters.
  { ensenso_camera_msgs::Parameter::MATCHING_METHOD,
    { ParameterType::String, { itmDisparityMap, itmStereoMatching, itmMethod } } },
  { ensenso_camera_msgs::Parameter::MINIMUM_DISPARITY,
    { ParameterType::Number, { itmDisparityMap, itmStereoMatching, itmMinimumDisparity } } },
  { ensenso_camera_msgs::Parameter::NUMBER_OF_DISPARITIES,
    { ParameterType::Number, { itmDisparityMap, itmStereoMatching, itmNumberOfDisparities } } },
  { ensenso_camera_msgs::Parameter::MEASUREMENT_VOLUME_NEAR,
    { ParameterType::Number, { itmDisparityMap, itmMeasurementVolume, itmNear, itmLeftBottom, "\2" } } },
  { ensenso_camera_msgs::Parameter::MEASUREMENT_VOLUME_FAR,
    { ParameterType::Number, { itmDisparityMap, itmMeasurementVolume, itmFar, itmLeftBottom, "\2" } } },
  { ensenso_camera_msgs::Parameter::UNIQUENESS_RATIO,
    { ParameterType::Number, { itmDisparityMap, itmPostProcessing, itmUniquenessRatio } } },
  { ensenso_camera_msgs::Parameter::SCALING, { ParameterType::Number, { itmDisparityMap, itmScaling } } },
  { ensenso_camera_msgs::Parameter::PADDING,
    { ParameterType::Bool, { itmDisparityMap, itmStereoMatching, itmPadding } } },
};

inline bool parameterExists(std::string const& key)
{
  return parameterInformation.count(key) > 0;
}
