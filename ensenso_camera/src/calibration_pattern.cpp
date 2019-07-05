#include "ensenso_camera/calibration_pattern.h"

#include "ensenso_camera/conversion.h"

template <typename messageType>
ChessboardPattern<messageType>::ChessboardPattern(NxLibItem const& node)
{
  thickness = node[itmThickness].asDouble() / ensenso_conversion::conversionFactor;
  gridSpacing = node[itmGridSpacing].asDouble() / ensenso_conversion::conversionFactor;
  gridSizeX = node[itmGridSize][0].asInt();
  gridSizeY = node[itmGridSize][1].asInt();
}

template <typename messageType>
ChessboardPattern<messageType>::ChessboardPattern(messageType const& message)
{
  thickness = message.thickness;
  gridSpacing = message.grid_spacing;
  gridSizeX = message.grid_size_x;
  gridSizeY = message.grid_size_y;
}

template <typename messageType>
void ChessboardPattern<messageType>::readMetaDataFromMessage(messageType const& message)
{
  thickness = message.thickness;
  gridSpacing = message.grid_spacing;
  gridSizeX = message.grid_size_x;
  gridSizeY = message.grid_size_y;
}

template <typename messageType>
void ChessboardPattern<messageType>::writeMetaDataToMessage(messageType& message)
{
  message.thickness = thickness;
  message.grid_spacing = gridSpacing;
  message.grid_size_x = gridSizeX;
  message.grid_size_y = gridSizeY;
}

template <typename messageType>
void ChessboardPattern<messageType>::writeMetaDataToNxLib(NxLibItem const& node)
{
  node[itmPattern][itmThickness] = thickness * ensenso_conversion::conversionFactor;
  node[itmPattern][itmGridSpacing] = gridSpacing * ensenso_conversion::conversionFactor;
  node[itmPattern][itmGridSize][0] = gridSizeX;
  node[itmPattern][itmGridSize][1] = gridSizeY;
}

template <typename messageType>
messageType ChessboardPattern<messageType>::toRosMessage() const
{
  messageType rosMsg;
  rosMsg.grid_size_x = gridSizeX;
  rosMsg.grid_size_y = gridSizeY;
  rosMsg.grid_spacing = gridSpacing;
  rosMsg.thickness = thickness;
  return rosMsg;
}

MonoCalibrationPattern::MonoCalibrationPattern(NxLibItem const& node) : ChessboardPattern(node)
{
}

MonoCalibrationPattern::MonoCalibrationPattern(ensenso_camera_msgs::MonoCalibrationPattern const& message)
  : ChessboardPattern(message)
{
  readFromMessage(message);
}

void MonoCalibrationPattern::readFromMessage(ensenso_camera_msgs::MonoCalibrationPattern const& message)
{
  points.clear();
  points.insert(points.begin(), message.points.begin(), message.points.end());
}

void MonoCalibrationPattern::writeToMessage(ensenso_camera_msgs::MonoCalibrationPattern& message)
{
  ChessboardPattern::writeMetaDataToMessage(message);

  message.points.clear();
  message.points.insert(message.points.begin(), points.begin(), points.end());
}

void MonoCalibrationPattern::writeToNxLib(NxLibItem const& node)
{
  ChessboardPattern::writeMetaDataToNxLib(node);
  for (size_t i = 0; i < points.size(); i++)
  {
    node[itmPoints][i][0] = points[i].x;
    node[itmPoints][i][1] = points[i].y;
  }
}

ensenso_camera_msgs::MonoCalibrationPattern MonoCalibrationPattern::toRosMsg() const
{
  ensenso_camera_msgs::MonoCalibrationPattern rosMonoPattern = ChessboardPattern::toRosMessage();

  for (auto const& point : points)
  {
    rosMonoPattern.points.push_back(point);
  }

  return rosMonoPattern;
}

void StereoCalibrationPattern::writeToMessage(ensenso_camera_msgs::StereoCalibrationPattern& message) const
{
  message = toRosMsg();
}

void StereoCalibrationPattern::readFromMessage(ensenso_camera_msgs::StereoCalibrationPattern const& message)
{
  leftPoints.clear();
  leftPoints.insert(leftPoints.begin(), message.left_points.begin(), message.left_points.end());
  rightPoints.clear();
  rightPoints.insert(rightPoints.begin(), message.right_points.begin(), message.right_points.end());
}

void StereoCalibrationPattern::writeToNxLib(NxLibItem const& node, bool right)
{
  ChessboardPattern::writeMetaDataToNxLib(node);
  auto& points = leftPoints;
  if (right)
  {
    points = rightPoints;
  }

  for (size_t i = 0; i < points.size(); i++)
  {
    node[itmPoints][i][0] = points[i].x;
    node[itmPoints][i][1] = points[i].y;
  }
}

StereoCalibrationPattern::StereoCalibrationPattern(NxLibItem const& node) : ChessboardPattern(node)
{
}

StereoCalibrationPattern::StereoCalibrationPattern(ensenso_camera_msgs::StereoCalibrationPattern const& message)
  : ChessboardPattern(message)
{
  readFromMessage(message);
}

ensenso_camera_msgs::StereoCalibrationPattern StereoCalibrationPattern::toRosMsg() const
{
  ensenso_camera_msgs::StereoCalibrationPattern rosStereoPattern = ChessboardPattern::toRosMessage();

  for (auto const& point : leftPoints)
  {
    rosStereoPattern.left_points.push_back(point);
  }
  for (auto const& point : rightPoints)
  {
    rosStereoPattern.right_points.push_back(point);
  }

  return rosStereoPattern;
}
