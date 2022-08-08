#pragma once

#include "ensenso_camera/ros2_node_handle.h"

#ifdef ROS2 /**********************************************************************************************************/
#include <boost/math/special_functions/round.hpp>
#include <rclcpp/create_timer.hpp>
#include "rclcpp/rclcpp.hpp"

namespace ensenso
{
namespace ros
{
using Rate = ::rclcpp::Rate;
using Time = ::rclcpp::Time;
using Timer = ::rclcpp::TimerBase::SharedPtr;

inline ::rclcpp::Duration durationFromSeconds(double d)
{
  return ::rclcpp::Duration(::std::chrono::duration<double>(d));
}

inline Time now(ensenso::ros::NodeHandle const& nh)
{
  return nh->node()->get_clock()->now();
}

inline void sleep(double t)
{
  rclcpp::Rate loop_rate(t);
  loop_rate.sleep();
}

inline Time timeFromSeconds(double t)
{
  // Implementation source: http://docs.ros.org/en/latest/api/rostime/html/impl_2time_8h_source.html#l00076
  int64_t sec64 = static_cast<int64_t>(floor(t));
  if (sec64 < 0 || sec64 > ::std::numeric_limits<uint32_t>::max())
    throw ::std::runtime_error("Time is out of dual 32-bit range");
  uint32_t sec = static_cast<uint32_t>(sec64);
  uint32_t nsec = static_cast<uint32_t>(boost::math::round((t - sec) * 1e9));
  // avoid rounding errors
  sec += (nsec / 1000000000ul);
  nsec %= 1000000000ul;
  return ::rclcpp::Time(sec, nsec);
}
}  // namespace ros
}  // namespace ensenso

#define TIMER_CALLBACK_DECLARATION_ARGS

#define TIMER_CALLBACK_DEFINITION_ARGS

#define CREATE_TIMER(nh, duration_in_seconds, callback_ref, object_ptr)                                                \
  ::rclcpp::create_timer(nh->get_base_interface(), nh->get_timers_interface(), nh->get_clock_interface()->get_clock(), \
                         ensenso::ros::durationFromSeconds(duration_in_seconds),                                       \
                         ensenso::std::bind(callback_ref, object_ptr));

#else /***ROS1*********************************************************************************************************/
#include <ros/ros.h>

namespace ensenso
{
namespace ros
{
using Rate = ::ros::Rate;
using Time = ::ros::Time;
using Timer = ::ros::Timer;
using TimerEvent = ::ros::TimerEvent;

inline ::ros::Duration durationFromSeconds(double d)
{
  return ::ros::Duration(d);
}

inline ::ros::Time now(ensenso::ros::NodeHandle const& nh)
{
  // (void)nh;
  return ::ros::Time::now();
}

inline void sleep(double t)
{
  ::ros::Duration(t).sleep();
}

inline ::ros::Time timeFromSeconds(double t)
{
  return ::ros::Time(t);
}
}  // namespace ros
}  // namespace ensenso

#define TIMER_CALLBACK_DECLARATION_ARGS ensenso::ros::TimerEvent const& timerEvent = ensenso::ros::TimerEvent()

#define TIMER_CALLBACK_DEFINITION_ARGS ensenso::ros::TimerEvent const& timerEvent

#define CREATE_TIMER(nh, duration_in_seconds, callback_ref, object_ptr)                                                \
  nh.createTimer(ensenso::ros::durationFromSeconds(duration_in_seconds), callback_ref, object_ptr)
#endif
