#pragma once

#include "ensenso_camera_msgs/StereoCalibrationPattern.h"
#include "ensenso_camera_msgs/MonoCalibrationPattern.h"
#include "ensenso_camera_msgs/ImagePoint.h"

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

class MonoCalibrationPattern : CalibrationPattern<ensenso_camera_msgs::MonoCalibrationPattern>
{
public:
  std::vector<ensenso_camera_msgs::ImagePoint> points;

public:
  explicit MonoCalibrationPattern(NxLibItem const& node);
  explicit MonoCalibrationPattern(ensenso_camera_msgs::MonoCalibrationPattern const& message);

  void readFromMessage(ensenso_camera_msgs::MonoCalibrationPattern const& message);
  void writeToMessage(ensenso_camera_msgs::MonoCalibrationPattern& message);
  void writeToNxLib(NxLibItem const& node);
  ensenso_camera_msgs::MonoCalibrationPattern toRosMsg() const;
};

class StereoCalibrationPattern : CalibrationPattern<ensenso_camera_msgs::StereoCalibrationPattern>
{
public:
  std::vector<ensenso_camera_msgs::ImagePoint> leftPoints;
  std::vector<ensenso_camera_msgs::ImagePoint> rightPoints;

public:
  explicit StereoCalibrationPattern(NxLibItem const& node);
  explicit StereoCalibrationPattern(ensenso_camera_msgs::StereoCalibrationPattern const& message);

  void writeToMessage(ensenso_camera_msgs::StereoCalibrationPattern& message) const;
  void readFromMessage(ensenso_camera_msgs::StereoCalibrationPattern const& message);
  void writeToNxLib(NxLibItem const& node, bool right = false);
  ensenso_camera_msgs::StereoCalibrationPattern toRosMsg() const;
};
