#include "ensenso_camera/nodelet_mono.h"

#include <pluginlib/class_list_macros.h>
#include <string>

#include "nxLib.h"

#include "ensenso_camera/helper.h"

namespace ensenso_camera
{
void NodeletMono::onInit()
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

  std::string serial, fileCameraPath, cameraFrame, targetFrame, linkFrame;
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

    // For long serial numbers that exceed the range of an integer with 32-bits, we can only determine the serial number
    // if it is passed as a numeric string. For this you have to append a "!" to the number so that it can be
    // interpreted as a string.

    NxLibItem cameraNode = NxLibItem()[itmCameras][itmBySerialNo][serial];
    if (!cameraNode.exists())
    {
      NODELET_WARN("The serial of the camera has been too long and was interpreted as an 32-bit integer and exceeds "
                   "its "
                   "length. Please append a \"!\" to "
                   "the number. E.g. \'_serial:=1234567\' to \'_serial:=1234567!\', so it can be "
                   "interpreted as a numerical "
                   "string.");
      NODELET_ERROR("Could not find any camera. Shutting down.");
      nxLibFinalize();
      exit(EXIT_FAILURE);
    }
  }

  // Check if string contains the "!" and delete it.
  std::size_t found = serial.find('!');
  if (found != std::string::npos)
  {
    serial.erase(found);
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

  // We can only check the type with file cameras, when we create file cameras, so for now we can only check whether the
  // current physical camera is of the right type
  if (fileCameraPath.empty())
  {
    std::string type = NxLibItem()[itmCameras][itmBySerialNo][serial][itmType].asString();
    std::string const neededType = "Monocular";
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

  nhLocal.getParam("target_frame", targetFrame);
  nhLocal.getParam("link_frame", linkFrame);
  if (targetFrame.empty() && linkFrame.empty())
  {
    // camera frame is also the global workspace
    targetFrame = cameraFrame;
    linkFrame = cameraFrame;
  }
  if (!targetFrame.empty() && linkFrame.empty())
  {
    // There is no link in between link and target frame. So the global NxLib Trafo must be identity.
    linkFrame = targetFrame;
  }

  camera = make_unique<MonoCamera>(nh, serial, fileCameraPath, cameraIsFixed, cameraFrame, targetFrame, linkFrame);
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
}

NodeletMono::~NodeletMono()
{
  camera->close();
  nxLibFinalize();
}

}  // namespace ensenso_camera

PLUGINLIB_EXPORT_CLASS(ensenso_camera::NodeletMono, nodelet::Nodelet)
