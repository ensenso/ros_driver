#pragma once

#include "ensenso_camera/ros2_node_handle.h"

#include <string>

// ---------------------------------------------------------------------------------------------------------------------
// Macros for unifying ROS1 and ROS2 logging
// ---------------------------------------------------------------------------------------------------------------------
//
// To print a console message, instead of using ROS_INFO(), we use RCLCPP_INFO() and its various cousins. The key
// difference is that RCLCPP_INFO() takes a Logger object as the first argument.
//
// ROS1
// http://docs.ros.org/en/noetic/api/rosconsole/html/macros__generated_8h_source.html#l00162
// ROS_INFO("%s", msg.data.c_str());
//
// ROS2
// https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html#a44818c8a1fd292cd0e81759e956765d7
// RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());
//
// Non-node loggers can also be created that use a specific name.
// see: https://answers.ros.org/question/361542/ros-2-how-to-create-a-non-node-logger/
// see: https://docs.ros2.org/dashing/api/rclcpp/namespacerclcpp.html#ae7295751947c08312aa69f45fd673171
// --> rclcpp::get_logger("AnyLoggerName")

// ---------------------------------------------------------------------------------------------------------------------
// Macros for unifying ROS1 and ROS2 logging - General
// ---------------------------------------------------------------------------------------------------------------------

#define DEFAULT_LOGGER "ensenso_camera"

// ---------------------------------------------------------------------------------------------------------------------
// Macros for unifying ROS1 and ROS2 logging - DEBUG
// ---------------------------------------------------------------------------------------------------------------------

#ifdef ROS2 /**********************************************************************************************************/
#include "rclcpp/rclcpp.hpp"

template <typename... T>
void ENSENSO_DEBUG(const char* arg, T... args)
{
  RCLCPP_DEBUG(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_DEBUG(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_DEBUG(nh->get_logging_interface()->get_logger(), arg, args...);
}

template <typename... T>
void ENSENSO_DEBUG_NAMED(std::string name, T... args)
{
  RCLCPP_DEBUG(rclcpp::get_logger(name), args...);
}

template <typename... T>
void ENSENSO_DEBUG_ONCE(const char* arg, T... args)
{
  RCLCPP_DEBUG_ONCE(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_DEBUG_ONCE(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_DEBUG_ONCE(nh->get_logging_interface()->get_logger(), arg, args...);
}

#else /***ROS1*********************************************************************************************************/
#include <ros/ros.h>

template <typename... T>
void ENSENSO_DEBUG(T... args)
{
  ROS_DEBUG(args...);
}

template <typename... T>
void ENSENSO_DEBUG(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_DEBUG(args...);
}

template <typename... T>
void ENSENSO_DEBUG_NAMED(std::string name, T... args)
{
  ROS_DEBUG_NAMED(name, args...);
}

template <typename... T>
void ENSENSO_DEBUG_ONCE(T... args)
{
  ROS_DEBUG_ONCE(args...);
}

template <typename... T>
void ENSENSO_DEBUG_ONCE(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_DEBUG_ONCE(args...);
}
#endif

// ---------------------------------------------------------------------------------------------------------------------
// Macros for unifying ROS1 and ROS2 logging - INFO
// ---------------------------------------------------------------------------------------------------------------------

#ifdef ROS2
template <typename... T>
void ENSENSO_INFO(const char* arg, T... args)
{
  RCLCPP_INFO(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_INFO(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_INFO(nh->get_logging_interface()->get_logger(), arg, args...);
}

template <typename... T>
void ENSENSO_INFO_NAMED(std::string name, T... args)
{
  RCLCPP_INFO(rclcpp::get_logger(name), args...);
}

template <typename... T>
void ENSENSO_INFO_ONCE(const char* arg, T... args)
{
  RCLCPP_INFO_ONCE(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_INFO_ONCE(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_INFO_ONCE(nh->get_logging_interface()->get_logger(), arg, args...);
}
#else
template <typename... T>
void ENSENSO_INFO(T... args)
{
  ROS_INFO(args...);
}

template <typename... T>
void ENSENSO_INFO(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_INFO(args...);
}

template <typename... T>
void ENSENSO_INFO_NAMED(std::string name, T... args)
{
  ROS_INFO_NAMED(name, args...);
}

template <typename... T>
void ENSENSO_INFO_ONCE(T... args)
{
  ROS_INFO_ONCE(args...);
}

template <typename... T>
void ENSENSO_INFO_ONCE(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_INFO_ONCE(args...);
}
#endif

// ---------------------------------------------------------------------------------------------------------------------
// Macros for unifying ROS1 and ROS2 logging - WARN
// ---------------------------------------------------------------------------------------------------------------------

#ifdef ROS2
template <typename... T>
void ENSENSO_WARN(const char* arg, T... args)
{
  RCLCPP_WARN(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_WARN(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_WARN(nh->get_logging_interface()->get_logger(), arg, args...);
}

template <typename... T>
void ENSENSO_WARN_NAMED(std::string name, T... args)
{
  RCLCPP_WARN(rclcpp::get_logger(name), args...);
}

template <typename... T>
void ENSENSO_WARN_ONCE(const char* arg, T... args)
{
  RCLCPP_WARN_ONCE(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_WARN_ONCE(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_WARN_ONCE(nh->get_logging_interface()->get_logger(), arg, args...);
}
#else
template <typename... T>
void ENSENSO_WARN(T... args)
{
  ROS_WARN(args...);
}

template <typename... T>
void ENSENSO_WARN(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_WARN(args...);
}

template <typename... T>
void ENSENSO_WARN_NAMED(std::string name, T... args)
{
  ROS_WARN_NAMED(name, args...);
}

template <typename... T>
void ENSENSO_WARN_ONCE(T... args)
{
  ROS_WARN_ONCE(args...);
}

template <typename... T>
void ENSENSO_WARN_ONCE(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_WARN_ONCE(args...);
}
#endif

// ---------------------------------------------------------------------------------------------------------------------
// Macros for unifying ROS1 and ROS2 logging - ERROR
// ---------------------------------------------------------------------------------------------------------------------

#ifdef ROS2
template <typename... T>
void ENSENSO_ERROR(const char* arg, T... args)
{
  RCLCPP_ERROR(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_ERROR(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_ERROR(nh->get_logging_interface()->get_logger(), arg, args...);
}

template <typename... T>
void ENSENSO_ERROR_NAMED(std::string name, T... args)
{
  RCLCPP_ERROR(rclcpp::get_logger(name), args...);
}

template <typename... T>
void ENSENSO_ERROR_ONCE(const char* arg, T... args)
{
  RCLCPP_ERROR_ONCE(rclcpp::get_logger(DEFAULT_LOGGER), arg, args...);
}

template <typename... T>
void ENSENSO_ERROR_ONCE(ensenso::ros::NodeHandle nh, const char* arg, T... args)
{
  RCLCPP_ERROR_ONCE(nh->get_logging_interface()->get_logger(), arg, args...);
}
#else
template <typename... T>
void ENSENSO_ERROR(T... args)
{
  ROS_ERROR(args...);
}

template <typename... T>
void ENSENSO_ERROR(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_ERROR(args...);
}

template <typename... T>
void ENSENSO_ERROR_NAMED(std::string name, T... args)
{
  ROS_ERROR_NAMED(name, args...);
}

template <typename... T>
void ENSENSO_ERROR_ONCE(T... args)
{
  ROS_ERROR_ONCE(args...);
}

template <typename... T>
void ENSENSO_ERROR_ONCE(ensenso::ros::NodeHandle nh, T... args)
{
  (void)nh;
  ROS_ERROR_ONCE(args...);
}
#endif
