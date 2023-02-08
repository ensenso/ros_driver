#pragma once

#include "ensenso_camera/ros2/node_handle.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <atomic>
#include <thread>
#include <vector>

namespace ensenso_camera
{
class VirtualObjectHandler
{
public:
  VirtualObjectHandler(ensenso::ros::NodeHandle& nh, const std::string& filename, const std::string& objectsFrame,
                       const std::string& linkFrame, const std::string& markerTopic, double markerPublishRate);
  ~VirtualObjectHandler();

  void updateObjectLinks();

private:
  std::thread markerThread;                    ///< Background thread used to publish object markers
  std::atomic_bool stopMarkerThread{ false };  ///< Flag to indicate markerThread should stop

  /// Original object transforms in the base frame
  std::vector<tf2::Transform> originalTransforms;
  std::vector<std::string> objectFrames;

  std::string objectsFrame;  ///< Frame in which objects are defined
  std::string linkFrame;     ///< Link frame where NxLib expects the objects to be defined

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener{ tfBuffer };
};
}  // namespace ensenso_camera
