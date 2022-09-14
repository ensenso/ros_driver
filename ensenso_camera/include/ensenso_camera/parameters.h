#pragma once

#include "ensenso_camera/ros2/namespace.h"

#include "ensenso_camera/ros2/ensenso_msgs/parameter.h"

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
 * Map ROS parameters (as defined in ensenso_camera_msgs/Parameter) to NxLib nodes.
 *
 * Note that there are other parameters that cannot be mapped directly to an NxLib node. These are handled in the
 * parameter reading and writing methods of the camera class.
 */
std::map<std::string, ParameterMapping> const parameterInformation{
  // NOLINT
  // Capture parameters.
  { ensenso::msg::Parameter::AUTO_EXPOSURE, { ParameterType::Bool, { itmCapture, itmAutoExposure } } },
  { ensenso::msg::Parameter::AUTO_GAIN, { ParameterType::Bool, { itmCapture, itmAutoGain } } },
  { ensenso::msg::Parameter::BINNING, { ParameterType::Number, { itmCapture, itmBinning } } },
  { ensenso::msg::Parameter::EXPOSURE, { ParameterType::Number, { itmCapture, itmExposure } } },
  { ensenso::msg::Parameter::FRONT_LIGHT, { ParameterType::Bool, { itmCapture, itmFrontLight } } },
  { ensenso::msg::Parameter::GAIN, { ParameterType::Number, { itmCapture, itmGain } } },
  { ensenso::msg::Parameter::GAIN_BOOST, { ParameterType::Bool, { itmCapture, itmGainBoost } } },
  { ensenso::msg::Parameter::HARDWARE_GAMMA, { ParameterType::Bool, { itmCapture, itmHardwareGamma } } },
  { ensenso::msg::Parameter::MAX_GAIN, { ParameterType::Number, { itmCapture, itmMaxGain } } },
  { ensenso::msg::Parameter::PIXEL_CLOCK, { ParameterType::Number, { itmCapture, itmPixelClock } } },
  { ensenso::msg::Parameter::PROJECTOR, { ParameterType::Bool, { itmCapture, itmProjector } } },
  { ensenso::msg::Parameter::TARGET_BRIGHTNESS, { ParameterType::Number, { itmCapture, itmTargetBrightness } } },
  { ensenso::msg::Parameter::TRIGGER_DELAY, { ParameterType::Number, { itmCapture, itmTriggerDelay } } },
  { ensenso::msg::Parameter::TRIGGER_MODE, { ParameterType::String, { itmCapture, itmTriggerMode } } },

  // Matching parameters.
  { ensenso::msg::Parameter::MATCHING_METHOD,
    { ParameterType::String, { itmDisparityMap, itmStereoMatching, itmMethod } } },
  { ensenso::msg::Parameter::MINIMUM_DISPARITY,
    { ParameterType::Number, { itmDisparityMap, itmStereoMatching, itmMinimumDisparity } } },
  { ensenso::msg::Parameter::NUMBER_OF_DISPARITIES,
    { ParameterType::Number, { itmDisparityMap, itmStereoMatching, itmNumberOfDisparities } } },
  { ensenso::msg::Parameter::MEASUREMENT_VOLUME_NEAR,
    { ParameterType::Number, { itmDisparityMap, itmMeasurementVolume, itmNear, itmLeftBottom, "\2" } } },
  { ensenso::msg::Parameter::MEASUREMENT_VOLUME_FAR,
    { ParameterType::Number, { itmDisparityMap, itmMeasurementVolume, itmFar, itmLeftBottom, "\2" } } },
  { ensenso::msg::Parameter::UNIQUENESS_RATIO,
    { ParameterType::Number, { itmDisparityMap, itmPostProcessing, itmUniquenessRatio } } },
  { ensenso::msg::Parameter::SCALING, { ParameterType::Number, { itmDisparityMap, itmScaling } } },
  { ensenso::msg::Parameter::PADDING, { ParameterType::Bool, { itmDisparityMap, itmStereoMatching, itmPadding } } },
};

inline bool parameterExists(std::string const& key)
{
  return parameterInformation.count(key) > 0;
}
