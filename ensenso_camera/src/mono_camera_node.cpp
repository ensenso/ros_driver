#include "ensenso_camera/mono_camera_node.h"

#include "ensenso_camera/camera_node.h"

REGISTER_NODE(ensenso_camera::MonoCamera)

namespace ensenso_camera
{
GENERATE_NODE_CLASS_IMPL(MonoCamera, valMonocular)
}  // namespace ensenso_camera
