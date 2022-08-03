#include "ensenso_camera/camera_node.h"

#include "ensenso_camera/mono_camera.h"
#include "ensenso_camera/nxlib_initialize_finalize.h"
#include "ensenso_camera/stereo_camera.h"

#include "ensenso_camera/ros2/logging.h"

#include <memory>

#include "nxLib.h"

namespace camera_node
{
void abortInit(ensenso::ros::NodeHandle& nh, std::string const& errorMsg)
{
  ENSENSO_ERROR(nh, "%s. Shutting down node.", errorMsg.c_str());
  // nxLibFinalize is implicitly invoked in NxLibInitializeFinalize destructor.
  exit(EXIT_FAILURE);
}

void initNxLib(ensenso::ros::NodeHandle& nh)
{
  ENSENSO_DEBUG(nh, "Initializing the NxLib...");
  try
  {
    NxLibInitializeFinalize::instance();
  }
  catch (NxLibException& e)
  {
    abortInit(nh, "Error while initializing the NxLib");
  }

  int tcpPort;
  if (ensenso::ros::get_parameter(nh, "tcp_port", tcpPort) && tcpPort != -1)
  {
    ENSENSO_DEBUG(nh, "Opening TCP port %d on the NxLib...", tcpPort);

    int openedPort;
    try
    {
      nxLibOpenTcpPort(tcpPort, &openedPort);
      ENSENSO_INFO(nh, "Opened TCP port %d on the NxLib.", openedPort);
    }
    catch (NxLibException& e)
    {
      abortInit(nh, "Error while opening TCP port (NxLib error message: " + e.getErrorText() + ")");
    }
  }

  int threads;
  if (ensenso::ros::get_parameter(nh, "threads", threads) && threads > 0)
  {
    NxLibItem()[itmParameters][itmThreads] = threads;
  }
}

std::string getSerialFromParameterServer(ensenso::ros::NodeHandle& nh)
{
  std::string serial;

  // Try to retrieve the serial as a string.
  if (ensenso::ros::get_parameter(nh, "serial", serial))
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
  if (ensenso::ros::get_parameter(nh, "serial", intSerial))
  {
    serial = std::to_string(intSerial);
  }

  NxLibItem cameraNode = NxLibItem()[itmCameras][itmBySerialNo][serial];
  if (!serial.empty() && !cameraNode.exists())
  {
    ENSENSO_WARN(nh,
                 "If the camera serial only consists of digits, its numerical value might be too large to be "
                 "interpreted as a 32-bit integer. Append an \"!\" to the serial so that it can be interpreted as a "
                 "string, e.g. _serial:=1234567890!. If you are using a launch file, just define the parameter's type "
                 "as string, e.g. type=\"string\".");
    abortInit(nh, "Could not find camera with serial " + serial);
  }

  // String is empty if no serial was given.
  return serial;
}

std::string getSerialOfFirstCamera(ensenso::ros::NodeHandle& nh, std::string const& cameraNodeType)
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
    abortInit(nh, "Could not find any camera");
  }

  return serial;
}

std::string getSerial(ensenso::ros::NodeHandle& nh, std::string const& nodeType)
{
  std::string serial = getSerialFromParameterServer(nh);

  if (!serial.empty())
  {
    return serial;
  }

  // No serial was given, get the serial of the first camera listed by the NxLib.
  return getSerialOfFirstCamera(nh, nodeType);
}

void loadCameraSettings(ensenso::ros::NodeHandle& nh, Camera& camera)
{
  std::string settingsFile;
  if (ensenso::ros::get_parameter(nh, "settings", settingsFile))
  {
    ENSENSO_DEBUG(nh, "Loading camera settings...");
    if (!camera.loadSettings(settingsFile, true))
    {
      abortInit(nh, "Failed to load the camera settings");
    }
  }
}

template <typename CameraType>
std::unique_ptr<CameraType> initCamera(ensenso::ros::NodeHandleWrapper& nhw, std::string const& nodeType)
{
  // Get the serial, either from the parameter server if the serial was given as parameter to the node or use the serial
  // of the first camera in the list of the NxLib. At this point, the serial either belongs to a mono or stereo camera
  // and the type matches the camera node's type.
  std::string serial = getSerial(nhw.getPrivateNodeHandle(), nodeType);

  CameraParameters params(nhw.getPrivateNodeHandle(), nodeType, serial);
  auto camera = ensenso::std::make_unique<CameraType>(nhw.getNodeHandle(), std::move(params));

  if (!camera->open())
  {
    abortInit(nhw.getNodeHandle(), "Failed to open the camera");
  }

  loadCameraSettings(nhw.getPrivateNodeHandle(), *camera);
  camera->init();

  return camera;
}

template std::unique_ptr<StereoCamera> initCamera<StereoCamera>(ensenso::ros::NodeHandleWrapper& nhw,
                                                                std::string const& nodeType);
template std::unique_ptr<MonoCamera> initCamera<MonoCamera>(ensenso::ros::NodeHandleWrapper& nhw,
                                                            std::string const& nodeType);

}  // namespace camera_node
