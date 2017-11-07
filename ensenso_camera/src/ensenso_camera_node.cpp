#include <ros/ros.h>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>

#include <cstdlib>

/**
 * Start the Ensenso camera driver as a standalone node.
 * Based on https://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ensenso_camera_node");

  nodelet::M_string remappings(ros::names::getRemappings());
  nodelet::V_string arguments;
  for (int i = 0; i < argc; i++)
  {
    arguments.push_back(argv[i]);
  }

  nodelet::Loader nodelet;
  nodelet.load(ros::this_node::getName(), "ensenso_camera/nodelet", remappings, arguments);

  ros::spin();

  return EXIT_SUCCESS;
}
