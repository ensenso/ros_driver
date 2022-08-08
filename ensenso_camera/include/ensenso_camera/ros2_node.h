#pragma once

#include "ensenso_camera/ros2_node_handle.h"

// ---------------------------------------------------------------------------------------------------------------------
// Code from ROS2 visibility_control.h
// ---------------------------------------------------------------------------------------------------------------------

#ifdef ROS2 /**********************************************************************************************************/
#ifndef ENSENSO_CAMERA__VISIBILITY_CONTROL_H_
#define ENSENSO_CAMERA__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ENSENSO_CAMERA_EXPORT __attribute__((dllexport))
#define ENSENSO_CAMERA_IMPORT __attribute__((dllimport))
#else
#define ENSENSO_CAMERA_EXPORT __declspec(dllexport)
#define ENSENSO_CAMERA_IMPORT __declspec(dllimport)
#endif
#ifdef ENSENSO_CAMERA_BUILDING_DLL
#define ENSENSO_CAMERA_PUBLIC ENSENSO_CAMERA_EXPORT
#else
#define ENSENSO_CAMERA_PUBLIC ENSENSO_CAMERA_IMPORT
#endif
#define ENSENSO_CAMERA_PUBLIC_TYPE ENSENSO_CAMERA_PUBLIC
#define ENSENSO_CAMERA_LOCAL
#else
#define ENSENSO_CAMERA_EXPORT __attribute__((visibility("default")))
#define ENSENSO_CAMERA_IMPORT
#if __GNUC__ >= 4
#define ENSENSO_CAMERA_PUBLIC __attribute__((visibility("default")))
#define ENSENSO_CAMERA_LOCAL __attribute__((visibility("hidden")))
#else
#define ENSENSO_CAMERA_PUBLIC
#define ENSENSO_CAMERA_LOCAL
#endif
#define ENSENSO_CAMERA_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ENSENSO_CAMERA__VISIBILITY_CONTROL_H_
#endif  // ROS2

// ---------------------------------------------------------------------------------------------------------------------
// Macros for unifying ROS1 nodelets and ROS2 nodes (which can be run as components)
// ---------------------------------------------------------------------------------------------------------------------

#ifdef ROS2 /**********************************************************************************************************/
#include "rclcpp_components/register_node_macro.hpp"

#define GENERATE_NODE_CLASS(ClassName)                                                                                 \
  class ClassName##Node : public rclcpp::Node                                                                          \
  {                                                                                                                    \
  public:                                                                                                              \
    ENSENSO_CAMERA_PUBLIC                                                                                              \
    explicit ClassName##Node(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());                              \
    ~ClassName##Node();                                                                                                \
                                                                                                                       \
  private:                                                                                                             \
    std::string cameraType;                                                                                            \
    std::unique_ptr<ClassName> camera;                                                                                 \
  };

#define GENERATE_NODE_CLASS_IMPL(ClassName, CameraType)                                                                \
  ClassName##Node::ClassName##Node(rclcpp::NodeOptions const& options)                                                 \
    : Node(#ClassName "Node", options), cameraType(CameraType)                                                         \
  {                                                                                                                    \
    DECLARE_NODE_PARAMETERS(this);                                                                                     \
                                                                                                                       \
    auto nh = create_node_handle(this);                                                                                \
    ensenso::ros::NodeHandleWrapper nhw = ensenso::ros::NodeHandleWrapper(std::move(nh));                              \
                                                                                                                       \
    camera_node::initNxLib(nhw.getPrivateNodeHandle());                                                                \
                                                                                                                       \
    camera = camera_node::initCamera<ClassName>(nhw, cameraType);                                                      \
  }                                                                                                                    \
                                                                                                                       \
  ClassName##Node::~ClassName##Node()                                                                                  \
  {                                                                                                                    \
    camera->close();                                                                                                   \
  }

#define NODE_PUBLIC_INTERFACE(ClassName)                                                                               \
  ENSENSO_CAMERA_PUBLIC                                                                                                \
  explicit ClassName##Node(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());                                \
  ~ClassName##Node();                                                                                                  \
  void onInit()

#define NODE_CTOR_IMPL_HEADER(ClassName)                                                                               \
  ClassName##Node::ClassName##Node(rclcpp::NodeOptions const& options) : Node(#ClassName "Node", options),

#define NODE_DTOR_IMPL_HEADER(ClassName) ClassName##Node::~ClassName##Node()

#define NODE_ON_INIT_IMPL_HEADER(ClassName) void ClassName##Node::onInit()

#define SINGLE_NODE_CLASS(ClassName) class ClassName : public rclcpp::Node
#define SINGLE_NODE_CTOR(ClassName, NodeName) ClassName() : Node(NodeName)

#define REGISTER_NODE(ClassName) RCLCPP_COMPONENTS_REGISTER_NODE(ClassName##Node)

#define DECLARE_NODE_PARAMETERS(node)                                                                                  \
  node->declare_parameter<std::string>("serial", "");                                                                  \
  node->declare_parameter<std::string>("settings", "");                                                                \
  node->declare_parameter<std::string>("file_camera_path", "");                                                        \
  node->declare_parameter<bool>("fixed", false);                                                                       \
  node->declare_parameter<int>("threads", -1);                                                                         \
  node->declare_parameter<std::string>("camera_frame", "");                                                            \
  node->declare_parameter<std::string>("target_frame", "");                                                            \
  node->declare_parameter<std::string>("link_frame", "");                                                              \
  node->declare_parameter<std::string>("robot_frame", "");                                                             \
  node->declare_parameter<std::string>("wrist_frame", "");                                                             \
  node->declare_parameter<int>("tcp_port", -1);                                                                        \
  node->declare_parameter<bool>("wait_for_camera", false);

namespace ensenso
{
namespace ros
{
struct NodeHandleWrapper
{
  NodeHandle nh;

  NodeHandleWrapper(NodeHandle nh) : nh(::std::move(nh))
  {
  }

  NodeHandle& getNodeHandle()
  {
    return nh;
  }

  NodeHandle& getPrivateNodeHandle()
  {
    return nh;
  }
};
}  // namespace ros
}  // namespace ensenso

#define CREATE_NODE_HANDLE_WRAPPER(name)                                                                               \
  rclcpp::Node::SharedPtr name##_ = shared_from_this();                                                                \
  ensenso::ros::NodeHandleWrapper name = ensenso::ros::NodeHandleWrapper(std::move(name##_));

#else /***ROS1*********************************************************************************************************/
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#define GENERATE_NODE_CLASS(ClassName)                                                                                 \
  class ClassName##Nodelet : public nodelet::Nodelet                                                                   \
  {                                                                                                                    \
  public:                                                                                                              \
    ClassName##Nodelet();                                                                                              \
    ~ClassName##Nodelet();                                                                                             \
    void onInit() override;                                                                                            \
                                                                                                                       \
  private:                                                                                                             \
    std::string cameraType;                                                                                            \
    std::unique_ptr<ClassName> camera;                                                                                 \
  };

#define GENERATE_NODE_CLASS_IMPL(ClassName, CameraType)                                                                \
  ClassName##Nodelet::ClassName##Nodelet() : cameraType(CameraType)                                                    \
  {                                                                                                                    \
  }                                                                                                                    \
                                                                                                                       \
  ClassName##Nodelet::~ClassName##Nodelet()                                                                            \
  {                                                                                                                    \
    camera->close();                                                                                                   \
  }                                                                                                                    \
                                                                                                                       \
  void ClassName##Nodelet::onInit()                                                                                    \
  {                                                                                                                    \
    CREATE_NODE_HANDLE_WRAPPER(nh)                                                                                     \
    camera_node::initNxLib(nh.getPrivateNodeHandle());                                                                 \
    camera = camera_node::initCamera<ClassName>(nh, cameraType);                                                       \
  }

#define NODE_PUBLIC_INTERFACE(ClassName)                                                                               \
public:                                                                                                                \
  ClassName##Nodelet();                                                                                                \
  ~ClassName##Nodelet();                                                                                               \
  void onInit() override

#define NODE_CTOR_IMPL_HEADER(ClassName) ClassName##Nodelet::ClassName##Nodelet() :

#define NODE_DTOR_IMPL_HEADER(ClassName) ClassName##Nodelet::~ClassName##Nodelet()

#define NODE_ON_INIT_IMPL_HEADER(ClassName) void ClassName##Nodelet::onInit()

#define SINGLE_NODE_CLASS(ClassName) class ClassName
#define SINGLE_NODE_CTOR(ClassName, NodeName) ClassName()

#define REGISTER_NODE(ClassName) PLUGINLIB_EXPORT_CLASS(ClassName##Nodelet, nodelet::Nodelet)

namespace ensenso
{
namespace ros
{
using NodeHandle = ::ros::NodeHandle;

struct NodeHandleWrapper
{
  NodeHandle nh;
  NodeHandle nhPrivate;

  NodeHandleWrapper(NodeHandle& nh, NodeHandle& nhPrivate) : nh(nh), nhPrivate(nhPrivate)
  {
  }
  NodeHandle& getNodeHandle()
  {
    return nh;
  }
  NodeHandle& getPrivateNodeHandle()
  {
    return nhPrivate;
  }
};
}  // namespace ros
}  // namespace ensenso

#define CREATE_NODE_HANDLE_WRAPPER(name)                                                                               \
  ensenso::ros::NodeHandleWrapper name = ensenso::ros::NodeHandleWrapper(getNodeHandle(), getPrivateNodeHandle());
#endif
