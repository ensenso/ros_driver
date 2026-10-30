#pragma once

#ifdef ROS2 /**********************************************************************************************************/

#define TF2_BUFFER_CTOR_ARGS(nh) nh->get_clock_interface()->get_clock()

inline std::unique_ptr<tf2_ros::Buffer> make_tf2_buffer(ensenso::ros::NodeHandle const& nh)
{
  return ensenso::std::make_unique<tf2_ros::Buffer>(TF2_BUFFER_CTOR_ARGS(nh));
}

inline std::unique_ptr<tf2_ros::TransformBroadcaster> make_tf2_broadcaster(ensenso::ros::NodeHandle const& nh)
{
  return ensenso::std::make_unique<tf2_ros::TransformBroadcaster>(nh->node());
}

#else /***ROS1*********************************************************************************************************/
#define TF2_BUFFER_CTOR_ARGS(nh)

inline std::unique_ptr<tf2_ros::Buffer> make_tf2_buffer(ensenso::ros::NodeHandle const& nh)
{
  return ensenso::std::make_unique<tf2_ros::Buffer>();
}

inline std::unique_ptr<tf2_ros::TransformBroadcaster> make_tf2_broadcaster(ensenso::ros::NodeHandle const& nh)
{
  return ensenso::std::make_unique<tf2_ros::TransformBroadcaster>();
}
#endif
