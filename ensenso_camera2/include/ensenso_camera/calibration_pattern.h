#pragma once

#include "ensenso_camera/ros2/namespace.h"

#include "ensenso_camera/ros2/ensenso_msgs/image_point.h"
#include "ensenso_camera/ros2/ensenso_msgs/mono_calibration_pattern.h"
#include "ensenso_camera/ros2/ensenso_msgs/stereo_calibration_pattern.h"

#include <vector>

#include "nxLib.h"

template <typename MessageType>
class CalibrationPattern
{
public:
  double thickness = 0;
  int gridSizeX = 0;
  int gridSizeY = 0;
  double gridSpacing = 0;

protected:
  explicit CalibrationPattern(MessageType const& message);
  explicit CalibrationPattern(NxLibItem const& node);

  void readMetaDataFromMessage(MessageType const& message);
  void writeMetaDataToMessage(MessageType& message);
  void writeMetaDataToNxLib(NxLibItem const& node);
  MessageType toRosMessage() const;
};

class MonoCalibrationPattern : CalibrationPattern<ensenso::msg::MonoCalibrationPattern>
{
public:
  std::vector<ensenso::msg::ImagePoint> points;

public:
  explicit MonoCalibrationPattern(NxLibItem const& node);
  explicit MonoCalibrationPattern(ensenso::msg::MonoCalibrationPattern const& message);

  void readFromMessage(ensenso::msg::MonoCalibrationPattern const& message);
  void writeToMessage(ensenso::msg::MonoCalibrationPattern& message);
  void writeToNxLib(NxLibItem const& node);
  ensenso::msg::MonoCalibrationPattern toRosMsg() const;
};

class StereoCalibrationPattern : CalibrationPattern<ensenso::msg::StereoCalibrationPattern>
{
public:
  std::vector<ensenso::msg::ImagePoint> leftPoints;
  std::vector<ensenso::msg::ImagePoint> rightPoints;

public:
  explicit StereoCalibrationPattern(NxLibItem const& node);
  explicit StereoCalibrationPattern(ensenso::msg::StereoCalibrationPattern const& message);

  void writeToMessage(ensenso::msg::StereoCalibrationPattern& message) const;
  void readFromMessage(ensenso::msg::StereoCalibrationPattern const& message);
  void writeToNxLib(NxLibItem const& node, bool right = false);
  ensenso::msg::StereoCalibrationPattern toRosMsg() const;
};
