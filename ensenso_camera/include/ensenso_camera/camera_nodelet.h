#pragma once

#include <nodelet/nodelet.h>

#include <memory>
#include <string>

namespace camera_nodelet
{
void initNxLib(ros::NodeHandle& nh);

template <typename CameraType>
std::unique_ptr<CameraType> initCamera(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate, std::string const& nodeType);

}  // namespace camera_nodelet
