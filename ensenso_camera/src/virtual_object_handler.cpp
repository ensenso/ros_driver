#include <ensenso_camera/virtual_object_handler.h>

#include <ensenso_camera/pose_utilities.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>

#include <nxLib.h>

using namespace ensenso_camera;

namespace
{
std::string readFile(const std::string& filename)
{
  std::ifstream file{ filename };
  if (!file.is_open() || !file.rdbuf())
  {
    throw std::runtime_error("Unable to read objects file");
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

VirtualObjectHandler::VirtualObjectHandler(const std::string& filename, const std::string& objectsFrame,
                                           const std::string& cameraFrame)
  : objectsFrame(objectsFrame), cameraFrame(cameraFrame)
{
  // Read the file contents and assign it to the objects tag
  auto objects = NxLibItem{ itmObjects };
  objects.setJson(readFile(filename));

  // Get the original transforms from file
  for (int i = 0; i < objects.count(); ++i)
  {
    originalTransforms.push_back(transformFromNxLib(objects[i][itmLink]));
  }
}

void VirtualObjectHandler::updateObjectLinks()
{
  // Exit early if we have no work to do
  if (originalTransforms.empty())
  {
    return;
  }

  // Find transform from the frame in which the objects were defined to the current optical frame
  tf2::Transform cameraTransform;
  try
  {
    cameraTransform = fromMsg(tfBuffer.lookupTransform(cameraFrame, objectsFrame, ros::Time(0)).transform);
  }
  catch (const tf2::TransformException& e)
  {
    ROS_WARN("Could not look up the virtual object pose due to the tf error: %s", e.what());
    return;
  }

  // Apply the transform to all of the original transforms
  for (size_t i = 0; i < originalTransforms.size(); ++i)
  {
    tf2::Transform objectTransform = cameraTransform * originalTransforms[i];
    NxLibItem objectLink = NxLibItem{ itmObjects }[static_cast<int>(i)][itmLink];
    writeTransformToNxLib(objectTransform.inverse(), objectLink);
  }
}
