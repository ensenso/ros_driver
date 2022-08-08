#pragma once

#ifdef ROS2 /**********************************************************************************************************/
#define TF2_BUFFER_CTOR_ARGS(nh) nh->get_clock_interface()->get_clock()
#define MAKE_TF2_BUFFER(nh) ensenso::std::make_unique<tf2_ros::Buffer>(TF2_BUFFER_CTOR_ARGS(nh))
#define MAKE_TF2_BROADCASTER(nh) ensenso::std::make_unique<tf2_ros::TransformBroadcaster>(nh->node())

#else /***ROS1*********************************************************************************************************/
#define TF2_BUFFER_CTOR_ARGS(nh)
#define MAKE_TF2_BUFFER(nh) ensenso::std::make_unique<tf2_ros::Buffer>()
#define MAKE_TF2_BROADCASTER(nh) ensenso::std::make_unique<tf2_ros::TransformBroadcaster>()
#endif
