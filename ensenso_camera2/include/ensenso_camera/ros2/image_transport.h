#pragma once

#include "ensenso_camera/ros2/namespace.h"
#include "ensenso_camera/ros2/node_handle.h"

#include "ensenso_camera/ros2/sensor_msgs/image.h"

#include <mutex>
#include <string>

#define NODE_CLASS_NAME TexturePointCloudNode
#define NODE_NAME "texture_point_cloud"

#ifdef ROS2 /**********************************************************************************************************/

#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>

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

inline ::image_transport::CameraPublisher create_camera_publisher(ensenso::ros::NodeHandle const& nh,
                                                                  ::std::string const& topic_name)
{
  return ::image_transport::create_camera_publisher(nh->node(), topic_name);
}

inline ::image_transport::Publisher create_publisher(ensenso::ros::NodeHandle const& nh,
                                                     ::std::string const& topic_name)
{
  return ::image_transport::create_publisher(nh->node(), topic_name);
}

template <typename T>
inline ::image_transport::Subscriber create_subscription(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                         void (T::*callback)(sensor_msgs::msg::ImageConstPtr const&),
                                                         T* object)
{
  /* https://github.com/ros-perception/image_common/issues/121 */
  return ::image_transport::create_subscription(nh->node(), topic_name,
                                                std::bind(callback, object, ::std::placeholders::_1),
                                                ensenso::image_transport::get_default_transport(nh));
}

}  // namespace image_transport
}  // namespace ensenso

#else /***ROS1*********************************************************************************************************/
#include <image_transport/image_transport.h>

namespace ensenso
{
namespace image_transport
{
inline ::image_transport::CameraPublisher create_camera_publisher(ensenso::ros::NodeHandle const& nh,
                                                                  ::std::string const& topic_name)
{
  ::image_transport::ImageTransport imageTransport(nh);
  return imageTransport.advertiseCamera(topic_name, 1);
}

inline ::image_transport::Publisher create_publisher(ensenso::ros::NodeHandle const& nh,
                                                     ::std::string const& topic_name)
{
  ::image_transport::ImageTransport imageTransport(nh);
  return imageTransport.advertise(topic_name, 1);
}

template <typename T>
inline ::image_transport::Subscriber create_subscription(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                         void (T::*callback)(sensor_msgs::msg::ImageConstPtr const&),
                                                         T* object)
{
  ::image_transport::ImageTransport imageTransport(nh);
  return imageTransport.subscribe(topic_name, 10, callback, object);
}
}  // namespace image_transport
}  // namespace ensenso
#endif
