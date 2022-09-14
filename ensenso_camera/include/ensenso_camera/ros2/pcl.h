#pragma once

#include "ensenso_camera/ros2/core.h"

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS2 /**********************************************************************************************************/
#include "ensenso_camera/ros2/namespace.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

USING_MSG(sensor_msgs, PointCloud2)

template <typename T>
using PointCloudPublisher = ensenso::ros::Publisher<sensor_msgs::msg::PointCloud2>;

template <typename T>
using PointCloudSubscription = ensenso::ros::Subscription<sensor_msgs::msg::PointCloud2>;

#define POINT_CLOUD_SUBSCRIPTION_CALLBACK(callback, T, arg_name)                                                       \
  callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> arg_name)

namespace ensenso
{
namespace pcl
{
template <typename T>
inline PointCloudPublisher<T> create_publisher(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                               int queue_size)
{
  return nh->node()->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, queue_size);
}

template <typename T, typename C, typename M>
inline PointCloudSubscription<T> create_subscription(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                     int queue_size, void (C::*callback)(M), C* object)
{
  return ensenso::ros::create_subscription<sensor_msgs::msg::PointCloud2>(nh, topic_name, queue_size, callback, object);
}
}  // namespace pcl
}  // namespace ensenso

#define STORE_POINT_CLOUD(point_cloud_msg, point_cloud_pcl)                                                            \
  /* In ROS2 we have to convert from sensor_msgs PointCloud2 to PCL PointCloud2. */                                    \
  pcl::fromROSMsg(*point_cloud_msg, *point_cloud_pcl);

template <typename T>
inline void publishPointCloud(ensenso::ros::Publisher<sensor_msgs::msg::PointCloud2> const& publisher,
                              std::unique_ptr<T> pointCloud)
{
  auto cloudMsg = ensenso::std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*pointCloud, *cloudMsg);
  publisher->publish(std::move(cloudMsg));
}

#else /***ROS1*********************************************************************************************************/
#include <pcl_ros/point_cloud.h>

template <typename T>
using PointCloudPublisher = ensenso::ros::Publisher<T>;

template <typename T>
using PointCloudSubscription = ensenso::ros::Subscription<T>;

#define POINT_CLOUD_SUBSCRIPTION_CALLBACK(callback, T, arg_name) callback(T::Ptr const& arg_name)

namespace ensenso
{
namespace pcl
{
template <typename T>
inline PointCloudPublisher<T> create_publisher(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                               int queue_size)
{
  return ensenso::ros::create_publisher<T>(nh, topic_name, queue_size);
}

template <typename T, typename C, typename M>
inline PointCloudSubscription<T> create_subscription(ensenso::ros::NodeHandle& nh, ::std::string const& topic_name,
                                                     int queue_size, void (C::*callback)(M), C* object)
{
  return ensenso::ros::create_subscription<T>(nh, topic_name, queue_size, callback, object);
}
}  // namespace pcl
}  // namespace ensenso

#define STORE_POINT_CLOUD(point_cloud_boost_ptr, point_cloud_std_ptr)                                                  \
  /* In ROS1 we only have to convert the point cloud ptr from boost to std. */                                         \
  point_cloud_std_ptr = ensenso::std::to_std(point_cloud_boost_ptr);

template <typename T>
inline void publishPointCloud(ensenso::ros::Publisher<T> const& publisher, std::unique_ptr<T> pointCloud)
{
  auto cloud_ = boost::shared_ptr<T>(std::move(pointCloud));
  publisher->publish(cloud_);
}
#endif
