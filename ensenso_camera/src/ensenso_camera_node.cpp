#include <ensenso_camera/stereo_camera_node.h>

#include <ensenso_camera/ros2_node_wrapper.h>

/**
 * Start the Ensenso camera driver as a standalone node.
 * Based on https://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html
 * and https://github.com/ros2/demos/blob/dashing/composition/src/manual_composition.cpp
 */
int main(int argc, char** argv)
{
  CAMERA_NODE_WRAPPER(StereoCamera, stereo_camera, node)
  return 0;
}
