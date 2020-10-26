#pragma once

#include <atomic>
#include <thread>

#include <vector>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

namespace ensenso_camera
{
class VirtualObjectHandler
{
public:
  VirtualObjectHandler(const std::string& filename, const std::string& objectsFrame, const std::string& linkFrame,
                       const std::string& markerTopic, double markerPublishRate);
  ~VirtualObjectHandler();

  void updateObjectLinks();

private:
  std::thread markerThread;                    ///< Background thread used to publish object markers
  std::atomic_bool stopMarkerThread{ false };  ///< Flag to indicate markerThread should stop

  /// Original object transforms in the base frame
  std::vector<tf2::Transform> originalTransforms;

  std::string objectsFrame;  ///< Frame in which objects are defined
  std::string linkFrame;     ///< Link frame where NxLib expects the objects to be defined

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener{ tfBuffer };
};
}  // namespace ensenso_camera
