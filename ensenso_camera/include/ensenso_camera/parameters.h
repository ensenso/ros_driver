#pragma once

#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "nxLib.h"

#include "ensenso_camera_msgs/Parameter.h"

/**
 * Possible types of parameters that are mapped to nodes of the NxLib.
 */
enum ParameterType
{
  boolType,
  numberType,  // Used for integers and floats.
  stringType
};

/**
 * Map ROS parameters (as defined in ensenso_camera_msgs/Parameter) to NxLib
 * nodes.
 * Note that there might be other parameters that are handled outside of the
 * NxLib. See the parameter reading and writing methods of the camera class.
 *
 * For each parameter key, we have the following information:
 *   * The type of the parameter
 *   * The path to the corresponding NxLib tree node. This path is given as a
 *     vector of sub items, relative to the camera's parameter node.
 */
std::map<std::string, std::pair<ParameterType, std::vector<std::string>>> const parameterInformation
{
  { ensenso_camera_msgs::Parameter::AUTO_EXPOSURE    , { boolType   , { itmCapture, itmAutoExposure     } } },
  { ensenso_camera_msgs::Parameter::AUTO_GAIN        , { boolType   , { itmCapture, itmAutoGain         } } },
  { ensenso_camera_msgs::Parameter::BINNING          , { numberType , { itmCapture, itmBinning          } } },
  { ensenso_camera_msgs::Parameter::EXPOSURE         , { numberType , { itmCapture, itmExposure         } } },
  { ensenso_camera_msgs::Parameter::FRONT_LIGHT      , { boolType   , { itmCapture, itmFrontLight       } } },
  { ensenso_camera_msgs::Parameter::GAIN             , { numberType , { itmCapture, itmGain             } } },
  { ensenso_camera_msgs::Parameter::GAIN_BOOST       , { boolType   , { itmCapture, itmGainBoost        } } },
  { ensenso_camera_msgs::Parameter::HARDWARE_GAMMA   , { boolType   , { itmCapture, itmHardwareGamma    } } },
  { ensenso_camera_msgs::Parameter::MAX_GAIN         , { numberType , { itmCapture, itmMaxGain          } } },
  { ensenso_camera_msgs::Parameter::PIXEL_CLOCK      , { numberType , { itmCapture, itmPixelClock       } } },
  { ensenso_camera_msgs::Parameter::PROJECTOR        , { boolType   , { itmCapture, itmProjector        } } },
  { ensenso_camera_msgs::Parameter::TARGET_BRIGHTNESS, { numberType , { itmCapture, itmTargetBrightness } } },
  { ensenso_camera_msgs::Parameter::TRIGGER_DELAY    , { numberType , { itmCapture, itmTriggerDelay     } } },
  { ensenso_camera_msgs::Parameter::TRIGGER_MODE     , { stringType , { itmCapture, itmTriggerMode      } } }
};

inline bool parameterExists(std::string const& key)
{
  return parameterInformation.count(key) > 0;
}

/**
 * Get the type of the parameter with the given key.
 */
inline ParameterType parameterType(std::string const& key)
{
  return parameterInformation.at(key).first;
}

/**
 * Get the NxLib node that contains the parameter with the given key.
 */
inline NxLibItem parameterNode(NxLibItem const& cameraNode, std::string const& key)
{
  auto node = cameraNode[itmParameters];
  for (auto const& path : parameterInformation.at(key).second)
  {
    node = node[path];
  }

  return node;
}
