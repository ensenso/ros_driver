#include "ensenso_camera/nodelet.h"

#include "ensenso_camera/helper.h"
#include "nxLib.h"

#include <pluginlib/class_list_macros.h>
#include <string>

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

  std::string serial, fileCameraPath, cameraFrame, targetFrame, linkFrame, robotFrame, wristFrame;
  bool cameraIsFixed;

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
    NxLibItem cameras = NxLibItem()[itmCameras][itmBySerialNo];
    // Try to find the first stereo camera.
    bool foundAppropriateCamera = false;
    for (int i = 0; i < cameras.count(); i++)
    {
      if (cameras[i][itmType].exists() && cameras[i][itmType].asString() == valStereo)
      {
        foundAppropriateCamera = true;
        serial = cameras[i].name();
        break;
      }
    }
    if (!foundAppropriateCamera)
    {
      NODELET_ERROR("Could not find any camera. Shutting down.");
      nxLibFinalize();
      exit(EXIT_FAILURE);
    }
  }

  nhLocal.param<std::string>("file_camera_path", fileCameraPath, "");

  // We can only check the type with file cameras, when we create file cameras, so for now we can only check whether the
  // current physical camera is of the right type
  if (fileCameraPath.empty())
  {
    std::string type = NxLibItem()[itmCameras][itmBySerialNo][serial][itmType].asString();
    std::string const neededType = valStereo;
    if (type != neededType)
    {
      NODELET_ERROR("The camera to be opened is of the wrong type %s. It should be %s.", type.c_str(),
                    neededType.c_str());
      nxLibFinalize();
      exit(EXIT_FAILURE);
    }
  }

  nhLocal.param("fixed", cameraIsFixed, false);

  if (!nhLocal.getParam("camera_frame", cameraFrame))
  {
    cameraFrame = serial + "_optical_frame";
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

  nhLocal.getParam("target_frame", targetFrame);
  nhLocal.getParam("link_frame", linkFrame);

  if (!targetFrame.empty() && linkFrame.empty())
  {
    linkFrame = targetFrame;
  }
  else if (targetFrame.empty() && linkFrame.empty())
  {
    targetFrame = cameraFrame;
    linkFrame = cameraFrame;
  }

  int captureTimeout;
  nhLocal.param("capture_timeout", captureTimeout, 0);

  camera = make_unique<StereoCamera>(nh, serial, fileCameraPath, cameraIsFixed, cameraFrame, targetFrame, robotFrame,
                                     wristFrame, linkFrame, captureTimeout);
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

  camera->startServers();
  camera->initTfPublishTimer();
  camera->initStatusTimer();
}

Nodelet::~Nodelet()
{
  camera->close();
  nxLibFinalize();
}

}  // namespace ensenso_camera

PLUGINLIB_EXPORT_CLASS(ensenso_camera::Nodelet, nodelet::Nodelet)
