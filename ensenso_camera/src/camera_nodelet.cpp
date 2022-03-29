#include "ensenso_camera/camera_nodelet.h"

#include "ensenso_camera/mono_camera.h"
#include "ensenso_camera/stereo_camera.h"

#include "nxLib.h"

#include <memory>

namespace camera_nodelet
{
void abortInit(std::string const& errorMsg)
{
  ROS_ERROR("%s. Shutting down nodelet.", errorMsg.c_str());
  try
  {
    nxLibFinalize();
  }
  catch (NxLibException& e)
  {
  }
  exit(EXIT_FAILURE);
}

void initNxLib(ros::NodeHandle& nh)
{
  ROS_DEBUG("Initializing the NxLib...");
  try
  {
    nxLibInitialize(true);
  }
  catch (NxLibException& e)
  {
    abortInit("Error while initializing the NxLib");
  }

  int threads;
  if (nh.getParam("threads", threads) && threads > 0)
  {
    NxLibItem()[itmParameters][itmThreads] = threads;
  }
}

std::string getSerialFromParameterServer(ros::NodeHandle& nh)
{
  std::string serial;

  // Try to retrieve the serial as a string.
  if (nh.getParam("serial", serial))
  {
    // Delete optional trailing "!" character.
    std::size_t pos = serial.find("!");
    if (pos != std::string::npos)
    {
      serial.erase(pos);
    }

    // Return the serial, because it might be the name for a file camera to be opened, which does not exists yet, and
    // checking for its existence must be skipped.
    return serial;
  }

  // Try to retrieve the serial as an integer, because rosparam automatically converts numeric strings to integer.
  int intSerial;
  if (nh.getParam("serial", intSerial))
  {
    serial = std::to_string(intSerial);
  }

  NxLibItem cameraNode = NxLibItem()[itmCameras][itmBySerialNo][serial];
  if (!serial.empty() && !cameraNode.exists())
  {
    ROS_WARN(
        "If the camera serial only consists of digits, its numerical value might be too large to be "
        "interpreted as a 32-bit integer. Append an \"!\" to the serial so that it can be interpreted as a string, "
        "e.g. _serial:=1234567890!. If you are using a launch file, just define the parameter's type as string, e.g. "
        "type=\"string\".");
    abortInit("Could not find camera with serial " + serial);
  }

  // String is empty if no serial was given.
  return serial;
}

std::string getSerialOfFirstCamera(ros::NodeHandle& nh, std::string const& cameraNodeType)
{
  std::string serial;

  bool foundAppropriateCamera = false;

  // Try to find the first camera that matches the type of the camera node.
  NxLibItem cameras = NxLibItem()[itmCameras][itmBySerialNo];
  for (int i = 0; i < cameras.count(); i++)
  {
    NxLibItem camera = cameras[i];
    NxLibItem cameraType = camera[itmType];
    if (camera[itmStatus][itmAvailable].asBool() && cameraType.exists() && cameraType.asString() == cameraNodeType)
    {
      foundAppropriateCamera = true;
      serial = camera.name();
      break;
    }
  }

  if (!foundAppropriateCamera)
  {
    abortInit("Could not find any camera");
  }

  return serial;
}

std::string getSerial(ros::NodeHandle& nh, std::string const& nodeType)
{
  std::string serial = getSerialFromParameterServer(nh);

  if (!serial.empty())
  {
    return serial;
  }

  // No serial was given, get the serial of the first camera listed by the NxLib.
  return getSerialOfFirstCamera(nh, nodeType);
}

void loadCameraSettings(ros::NodeHandle& nh, Camera& camera)
{
  std::string settingsFile;
  if (nh.getParam("settings", settingsFile))
  {
    ROS_DEBUG("Loading camera settings...");
    if (!camera.loadSettings(settingsFile, true))
    {
      abortInit("Failed to load the camera settings");
    }
  }
}

template <typename CameraType>
std::unique_ptr<CameraType> initCamera(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate, std::string const& nodeType)
{
  // Get the serial, either from the parameter server if the serial was given as parameter to the node or use the serial
  // of the first camera in the list of the NxLib. At this point, the serial either belongs to a mono or stereo camera
  // and the type matches the camera node's type.
  std::string serial = getSerial(nhPrivate, nodeType);

  CameraParameters params(nhPrivate, nodeType, serial);
  auto camera = ::make_unique<CameraType>(nh, std::move(params));

  if (!camera->open())
  {
    abortInit("Failed to open the camera");
  }

  loadCameraSettings(nhPrivate, *camera);
  camera->init();

  return camera;
}

template std::unique_ptr<StereoCamera> initCamera<StereoCamera>(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate,
                                                                std::string const& nodeType);
template std::unique_ptr<MonoCamera> initCamera<MonoCamera>(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate,
                                                            std::string const& nodeType);

}  // namespace camera_nodelet
