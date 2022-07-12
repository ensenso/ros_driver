#pragma once

#ifdef ROS2 /**********************************************************************************************************/
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#define CAMERA_NODE_WRAPPER(ClassName, _, __)                                                                          \
  rclcpp::init(argc, argv);                                                                                            \
  rclcpp::executors::SingleThreadedExecutor exec;                                                                      \
  rclcpp::NodeOptions options;                                                                                         \
  /* options.allow_undeclared_parameters(true); */                                                                     \
  /* options.automatically_declare_parameters_from_overrides(true); */                                                 \
                                                                                                                       \
  auto node = std::make_shared<ensenso_camera::ClassName##Node>(options);                                              \
                                                                                                                       \
  exec.add_node(node);                                                                                                 \
  exec.spin();                                                                                                         \
                                                                                                                       \
  rclcpp::shutdown();

#define SINGLE_NODE_WRAPPER(ClassName, NodeName)                                                                       \
  rclcpp::init(argc, argv);                                                                                            \
  rclcpp::spin(std::make_shared<ClassName>());                                                                         \
  rclcpp::shutdown();

#else /***ROS1*********************************************************************************************************/
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cstdlib>

#define CAMERA_NODE_WRAPPER(ClassName, ClassNameCamelCase, NodeName)                                                   \
  ros::init(argc, argv, "ensenso_camera_" #NodeName);                                                                  \
                                                                                                                       \
  nodelet::M_string remappings(ros::names::getRemappings());                                                           \
  nodelet::V_string arguments;                                                                                         \
  for (int i = 0; i < argc; i++)                                                                                       \
  {                                                                                                                    \
    arguments.push_back(argv[i]);                                                                                      \
  }                                                                                                                    \
                                                                                                                       \
  nodelet::Loader nodelet;                                                                                             \
  nodelet.load(ros::this_node::getName(), "ensenso_camera/" #ClassNameCamelCase "_node", remappings, arguments);       \
                                                                                                                       \
  ros::spin();

#define SINGLE_NODE_WRAPPER(ClassName, NodeName)                                                                       \
  ros::init(argc, argv, NodeName);                                                                                     \
  ClassName node;                                                                                                      \
  ros::spin();

#endif
