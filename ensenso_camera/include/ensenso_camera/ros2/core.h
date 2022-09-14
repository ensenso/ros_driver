#pragma once

#include "ensenso_camera/ros2/node_handle.h"

#ifdef ROS2 /**********************************************************************************************************/
#include "rclcpp/rclcpp.hpp"

#include <experimental/optional>

namespace ensenso
{
namespace std
{
using ::std::bind;
using ::std::function;
using ::std::make_shared;
using ::std::make_unique;
using ::std::experimental::optional;
}  // namespace std

namespace ros
{
template <typename T>
using Publisher = typename ::rclcpp::Publisher<T>::SharedPtr;

template <typename T>
using Subscription = typename ::rclcpp::Subscription<T>::SharedPtr;

template <typename T>
inline ensenso::ros::Publisher<T> create_publisher(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                   int queue_size)
{
  return nh->node()->create_publisher<T>(topic_name, queue_size);
}

template <typename T, typename C, typename M>
inline ensenso::ros::Subscription<T> create_subscription(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                         int queue_size, void (C::*callback)(M), C* object)
{
  return nh->node()->create_subscription<T>(topic_name, queue_size,
                                            ::std::bind(callback, object, ::std::placeholders::_1));
}

inline ::std::string get_node_name(NodeHandle& nh)
{
  return ::std::string(nh->get_base_interface()->get_name());
}

template <typename T>
inline bool get_parameter(NodeHandle& nh, const char* name, T& parameter)
{
  return nh->node()->get_parameter(name, parameter);
}

template <typename T>
inline bool get_parameter(NodeHandle& nh, ::std::string const& name, T& parameter)
{
  return nh->node()->get_parameter(name, parameter);
}

inline bool ok()
{
  return rclcpp::ok();
}
}  // namespace ros
}  // namespace ensenso

#include <rcpputils/asserts.hpp>
#define ENSENSO_ASSERT(cond) rcpputils::assert_true(cond)

#else /*****ROS1*******************************************************************************************************/
#include <boost/optional.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <memory>

#include <ros/ros.h>
namespace ensenso
{
namespace std
{
using boost::bind;
using boost::function;
using boost::make_shared;
using boost::optional;

template <typename T>
using shared_ptr = ::std::shared_ptr<T>;

template <typename T, typename... Args>
::std::unique_ptr<T> make_unique(Args&&... args)
{
  return ::std::unique_ptr<T>(new T(::std::forward<Args>(args)...));
}

/* Source for do_release and to_std:
 * https://stackoverflow.com/questions/6326757/conversion-from-boostshared-ptr-to-stdshared-ptr
 */
template <typename T>
void do_release(typename boost::shared_ptr<T> const&, T*)
{
}

template <typename T>
typename std::shared_ptr<T> to_std(typename boost::shared_ptr<T> const& p)
{
  return std::shared_ptr<T>(p.get(), boost::bind(&do_release<T>, p, _1));
}
}  // namespace std

namespace ros
{
template <typename T>
using Publisher = ::std::unique_ptr< ::ros::Publisher>;

template <typename T>
using Subscription = ::ros::Subscriber;

template <typename T>
inline ensenso::ros::Publisher<T> create_publisher(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                   int queue_size)
{
  return ensenso::std::make_unique< ::ros::Publisher>(nh.advertise<T>(topic_name, queue_size));
}

template <typename T, typename C, typename M>
inline ensenso::ros::Subscription<T> create_subscription(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                         int queue_size, void (C::*callback)(M), C* object)
{
  return nh.subscribe(topic_name, queue_size, callback, object);
}

inline ::std::string get_node_name(NodeHandle& nh)
{
  (void)nh;
  return ::ros::this_node::getName();
}

template <typename T>
inline bool get_parameter(NodeHandle& nh, const char* name, T& parameter)
{
  return nh.getParam(name, parameter);
}

template <typename T>
inline bool get_parameter(NodeHandle& nh, const ::std::string& name, T& parameter)
{
  return nh.getParam(name, parameter);
}

inline bool ok()
{
  return ros::ok();
}
}  // namespace ros
}  // namespace ensenso

#define ENSENSO_ASSERT(cond) ROS_ASSERT(cond)
#endif
