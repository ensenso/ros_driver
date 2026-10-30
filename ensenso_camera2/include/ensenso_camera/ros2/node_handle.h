#pragma once

#ifdef ROS2 /**********************************************************************************************************/
#include "rclcpp/rclcpp.hpp"

namespace ensenso
{
namespace ros
{
class NodeHandle_
{
public:
  NodeHandle_(rclcpp::Node* node) : _node(node)
  {
  }

  rclcpp::Node* node()
  {
    return _node;
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_base_interface()
  {
    return _node->get_node_base_interface();
  }

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr get_clock_interface()
  {
    return _node->get_node_clock_interface();
  }

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr get_logging_interface()
  {
    return _node->get_node_logging_interface();
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_parameters_interface()
  {
    return _node->get_node_parameters_interface();
  }

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_topics_interface()
  {
    return _node->get_node_topics_interface();
  }

  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr get_timers_interface()
  {
    return _node->get_node_timers_interface();
  }

  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr get_waitables_interface()
  {
    return _node->get_node_waitables_interface();
  }

private:
  rclcpp::Node* _node;
};

using NodeHandle = std::shared_ptr<NodeHandle_>;
}  // namespace ros
}  // namespace ensenso

#define create_node_handle(node) std::make_shared<ensenso::ros::NodeHandle_>(node)

#define CREATE_NODE_HANDLE(nh) nh = create_node_handle(this);

#else /***ROS1*********************************************************************************************************/
#include <ros/ros.h>

namespace ensenso
{
namespace ros
{
using NodeHandle = ::ros::NodeHandle;
}  // namespace ros
}  // namespace ensenso

#define CREATE_NODE_HANDLE(nh) static_assert(true, "")

#endif
