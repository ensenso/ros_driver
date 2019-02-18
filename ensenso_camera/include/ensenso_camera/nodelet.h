#pragma once

#include <nodelet/nodelet.h>

#include <memory>

#include "ensenso_camera/camera.h"

#include "ensenso_camera_msgs/SetSettings.h"
#include <ros/ros.h>

namespace ensenso_camera
{

class Nodelet : public nodelet::Nodelet
{
private:
  std::unique_ptr<Camera> camera;

public:
  void onInit() override;
  virtual ~Nodelet();
  bool setSettingsCallback(ensenso_camera_msgs::SetSettings::Request &req,
                            ensenso_camera_msgs::SetSettings::Response &res);

  ros::NodeHandle srv_nh;
  ros::ServiceServer service = srv_nh.advertiseService("ensenso_camera_node/set_ensenso_settings",
                                                       &ensenso_camera::Nodelet::setSettingsCallback, this);

};

}  // namespace ensenso_camera
