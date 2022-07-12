#include "ensenso_camera/stereo_camera_nodelet.h"

#include "ensenso_camera/camera_nodelet.h"

REGISTER_NODE(ensenso_camera::StereoCamera)

namespace ensenso_camera
{
GENERATE_NODE_CLASS_IMPL(StereoCamera, valStereo)
}  // namespace ensenso_camera
