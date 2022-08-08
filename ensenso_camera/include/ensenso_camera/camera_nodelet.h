#pragma once

#include "ensenso_camera/ros2_node.h"
#include "ensenso_camera/ros2_node_handle.h"

#include <memory>
#include <string>

namespace camera_node
{
void initNxLib(ensenso::ros::NodeHandle& nh);

template <typename CameraType>
std::unique_ptr<CameraType> initCamera(ensenso::ros::NodeHandleWrapper& nhw, std::string const& nodeType);

}  // namespace camera_node
