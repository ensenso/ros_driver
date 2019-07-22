#include "ensenso_camera/nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <string>

#include "nxLib.h"

#include "ensenso_camera/helper.h"

namespace ensenso_camera
{
void Nodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nhLocal = getPrivateNodeHandle();

  NODELET_DEBUG("Initializing the NxLib...");
  try
  {
    nxLibInitialize(true);
  }
  catch (NxLibException& e)
  {
    NODELET_ERROR("Error while initializing the NxLib. Shutting down.");
    exit(EXIT_FAILURE);
  }

  int threads;
  if (nhLocal.getParam("threads", threads))
  {
    NxLibItem()[itmParameters][itmThreads] = threads;
  }

  std::string serial, fileCameraPath, cameraFrame, targetFrame, robotFrame, wristFrame, linkedCameraFrame;
  bool cameraIsFixed, linked_camera_auto_exposure;

  if (!nhLocal.getParam("serial", serial))
  {
    // Try to get the serial as an integer, because rosparam sometimes
    // automatically converts the serials to numbers.
    int intSerial;
    if (nhLocal.getParam("serial", intSerial))
    {
      serial = std::to_string(intSerial);
    }
  }
  if (serial.empty())
  {
    // No serial specified. Try to use the first camera in the tree.
    NxLibItem cameras = NxLibItem()[itmCameras][itmBySerialNo];
    if (cameras.count() > 0)
    {
      serial = cameras[0].name();
    }
    else
    {
      NODELET_ERROR("Could not find any camera. Shutting down.");
      nxLibFinalize();
      exit(EXIT_FAILURE);
    }
  }

  nhLocal.param<std::string>("file_camera_path", fileCameraPath, "");
  nhLocal.param("fixed", cameraIsFixed, false);

  if (!nhLocal.getParam("camera_frame", cameraFrame))
  {
    cameraFrame = "ensenso_optical_frame";
  }
  if (!nhLocal.getParam("linked_camera_frame", linkedCameraFrame))
  {
    linkedCameraFrame = "linked_camera_frame";
  }
  if (!nhLocal.getParam("target_frame", targetFrame))
  {
    targetFrame = cameraFrame;
  }

  if (!nhLocal.getParam("linked_camera_auto_exposure", linked_camera_auto_exposure))
  {
    linked_camera_auto_exposure = false;
  }

  nhLocal.param<std::string>("robot_frame", robotFrame, "");
  nhLocal.param<std::string>("wrist_frame", wristFrame, "");
  if (cameraIsFixed && robotFrame.empty())
  {
    robotFrame = cameraFrame;
  }
  if (!cameraIsFixed && wristFrame.empty())
  {
    wristFrame = cameraFrame;
  }

  NODELET_DEBUG("Opening the camera '%s'...", serial.c_str());
  camera = make_unique<Camera>(nh, serial, fileCameraPath, cameraIsFixed, cameraFrame, targetFrame, robotFrame,
                               wristFrame, linkedCameraFrame, linked_camera_auto_exposure);
  if (!camera->open())
  {
    NODELET_ERROR("Failed to open the camera. Shutting down.");
    nxLibFinalize();
    exit(EXIT_FAILURE);
  }

  std::string settingsFile;
  if (nhLocal.getParam("settings", settingsFile))
  {
    NODELET_DEBUG("Loading camera settings...");
    if (!camera->loadSettings(settingsFile, true))
    {
      NODELET_ERROR("Failed to load the camera settings. Shutting down.");
      nxLibFinalize();
      exit(EXIT_FAILURE);
    }
  }

  std::string monocamSettingsFile;
  if (nhLocal.getParam("monocular_camera_settings", monocamSettingsFile))
  {
    NODELET_DEBUG("Loading monocular camera settings...");
    if (!camera->loadMonocularSettings(monocamSettingsFile))
    {
      NODELET_ERROR("Failed to load the monocular camera settings. Shutting down.");
      nxLibFinalize();
      exit(EXIT_FAILURE);
    }
  }

  camera->startServers();
}

Nodelet::~Nodelet()
{
  camera->close();
  nxLibFinalize();
}

bool Nodelet::setSettingsCallback(ensenso_camera_msgs::SetSettings::Request &req,
                          ensenso_camera_msgs::SetSettings::Response &res)
{
  NODELET_DEBUG("Loading camera settings...");
  camera->loadSettings(req.file_name);
  res.success = true;

  return true;
}

}  // namespace ensenso_camera


PLUGINLIB_EXPORT_CLASS(ensenso_camera::Nodelet, nodelet::Nodelet)
