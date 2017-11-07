#pragma once

#include <vector>

#include "nxLib.h"

#include <ensenso_camera_msgs/CalibrationPattern.h>
#include <ensenso_camera_msgs/ImagePoint.h>

struct CalibrationPattern
{
  double thickness = 0;
  int gridSizeX = 0;
  int gridSizeY = 0;
  double gridSpacing = 0;

  std::vector<ensenso_camera_msgs::ImagePoint> leftPoints;
  std::vector<ensenso_camera_msgs::ImagePoint> rightPoints;

  CalibrationPattern()
  {
  }

  explicit CalibrationPattern(NxLibItem const& node)
  {
    // NxLib patterns are specified in millimeters, output is in meters.
    thickness = node[itmThickness].asDouble() / 1000.0f;
    gridSpacing = node[itmGridSpacing].asDouble() / 1000.0f;
    gridSizeX = node[itmGridSize][0].asInt();
    gridSizeY = node[itmGridSize][1].asInt();
  }

  explicit CalibrationPattern(ensenso_camera_msgs::CalibrationPattern const& message)
  {
    readFromMessage(message);
  }

  void writeToMessage(ensenso_camera_msgs::CalibrationPattern* message) const
  {
    message->thickness = thickness;
    message->grid_spacing = gridSpacing;
    message->grid_size_x = gridSizeX;
    message->grid_size_y = gridSizeY;

    message->left_points.clear();
    message->left_points.insert(message->left_points.begin(), leftPoints.begin(), leftPoints.end());
    message->right_points.clear();
    message->right_points.insert(message->right_points.begin(), rightPoints.begin(), rightPoints.end());
  }

  void readFromMessage(ensenso_camera_msgs::CalibrationPattern const& message)
  {
    thickness = message.thickness;
    gridSpacing = message.grid_spacing;
    gridSizeX = message.grid_size_x;
    gridSizeY = message.grid_size_y;

    leftPoints.clear();
    leftPoints.insert(leftPoints.begin(), message.left_points.begin(), message.left_points.end());
    rightPoints.clear();
    rightPoints.insert(rightPoints.begin(), message.right_points.begin(), message.right_points.end());
  }

  void writeToNxLib(NxLibItem const& node, bool right = false)
  {
    node[itmPattern][itmThickness] = thickness * 1000;  // Data is in meters, NxLib expects it to be in millimeters.
    node[itmPattern][itmGridSpacing] = gridSpacing * 1000;
    node[itmPattern][itmGridSize][0] = gridSizeX;
    node[itmPattern][itmGridSize][1] = gridSizeY;

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
};
