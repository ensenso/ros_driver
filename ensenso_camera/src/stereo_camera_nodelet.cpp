#include "ensenso_camera/stereo_camera_nodelet.h"

#include "ensenso_camera/camera_nodelet.h"
#include "ensenso_camera/helper.h"

#include <pluginlib/class_list_macros.h>

namespace ensenso_camera
{
StereoCameraNodelet::StereoCameraNodelet() : cameraType(valStereo)
{
}

StereoCameraNodelet::~StereoCameraNodelet()
{
  camera->close();
}

void StereoCameraNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nhPrivate = getPrivateNodeHandle();

  camera_nodelet::initNxLib(nhPrivate);
  camera = camera_nodelet::initCamera<StereoCamera>(nh, nhPrivate, cameraType);
}

}  // namespace ensenso_camera

PLUGINLIB_EXPORT_CLASS(ensenso_camera::StereoCameraNodelet, nodelet::Nodelet)
