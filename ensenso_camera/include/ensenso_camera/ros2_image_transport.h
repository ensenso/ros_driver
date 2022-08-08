#pragma once

#ifdef ROS2 /**********************************************************************************************************/
#include "ensenso_camera/ros2_node_handle.h"

#include <image_transport/transport_hints.hpp>

// #include <string>

namespace ensenso
{
namespace image_transport
{
// https://github.com/ros-perception/image_common/blob/galactic/image_transport/src/image_transport.cpp
// https://github.com/ros-perception/image_common/blob/galactic/image_transport/include/image_transport/transport_hints.hpp
inline ::std::string get_default_transport(ensenso::ros::NodeHandle& nh)
{
  ::image_transport::TransportHints transportHints(nh->node());
  return transportHints.getTransport();
}
}  // namespace image_transport
}  // namespace ensenso

#define IMAGE_TRANSPORT_INIT(nh) static_assert(true, "")
#define IMAGE_TRANSPORT_CREATE_CAMERA_PUBLISHER(nh, topic) image_transport::create_camera_publisher(nh->node(), topic)
#define IMAGE_TRANSPORT_CREATE_PUBLISHER(nh, topic) image_transport::create_publisher(nh->node(), topic)
#define IMAGE_TRANSPORT_CREATE_SUBSCRIPTION(nh, topic, callback, object)                                               \
  /* https://github.com/ros-perception/image_common/issues/121 */                                                      \
  image_transport::create_subscription(nh->node(), topic, std::bind(callback, object, std::placeholders::_1),          \
                                       ensenso::image_transport::get_default_transport(nh))

#else /***ROS1*********************************************************************************************************/
#define IMAGE_TRANSPORT_INIT(nh) image_transport::ImageTransport imageTransport(nh)
#define IMAGE_TRANSPORT_CREATE_CAMERA_PUBLISHER(nh, topic) imageTransport.advertiseCamera(topic, 1)
#define IMAGE_TRANSPORT_CREATE_PUBLISHER(nh, topic) imageTransport.advertise(topic, 1)
#define IMAGE_TRANSPORT_CREATE_SUBSCRIPTION(nh, topic, callback, object)                                               \
  imageTransport.subscribe(topic, 10, callback, object)
#endif
