#include "ensenso_camera/mono_camera_nodelet.h"

#include "ensenso_camera/camera_nodelet.h"
#include "ensenso_camera/helper.h"

#include <pluginlib/class_list_macros.h>

namespace ensenso_camera
{
MonoCameraNodelet::MonoCameraNodelet() : cameraType(valMonocular)
{
}

MonoCameraNodelet::~MonoCameraNodelet()
{
  camera->close();
  nxLibFinalize();
}

void MonoCameraNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nhPrivate = getPrivateNodeHandle();

  camera_nodelet::initNxLib(nhPrivate);
  camera = camera_nodelet::initCamera<MonoCamera>(nh, nhPrivate, cameraType);
}

}  // namespace ensenso_camera

PLUGINLIB_EXPORT_CLASS(ensenso_camera::MonoCameraNodelet, nodelet::Nodelet)
