#pragma once

#include "ensenso_camera/ros2_core.h"

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS2 /**********************************************************************************************************/
#include "ensenso_camera/ros2_namespace.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

USING_MSG(sensor_msgs, PointCloud2)

#define POINT_CLOUD_PUBLISHER(T) ensenso::ros::Publisher<sensor_msgs::msg::PointCloud2>

#define POINT_CLOUD_SUBSCRIPTION(T)                                                                                    \
  /* In ROS2 we currently have to publish point clouds as sensor_msgs PointCloud2 */                                   \
  ensenso::ros::Subscription<sensor_msgs::msg::PointCloud2>

#define POINT_CLOUD_SUBSCRIPTION_CALLBACK(callback, T, arg_name)                                                       \
  callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> arg_name)

#define CREATE_POINT_CLOUD_PUBLISHER(nh, T, topic_name, queue_size)                                                    \
  nh->node()->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, queue_size)

#define CREATE_POINT_CLOUD_SUBSCRIPTION(nh, T, topic_name, qos, callback, object)                                      \
  CREATE_SUBSCRIPTION(nh, sensor_msgs::msg::PointCloud2, topic_name, qos, callback, object)

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

#define POINT_CLOUD_PUBLISHER(T) ensenso::ros::Publisher<T>

#define POINT_CLOUD_SUBSCRIPTION(T)                                                                                    \
  /* In ROS1 we can publish point clouds as pcl PointCloud2 */                                                         \
  ensenso::ros::Subscription<T>

#define POINT_CLOUD_SUBSCRIPTION_CALLBACK(callback, T, arg_name) callback(T::Ptr const& arg_name)

#define CREATE_POINT_CLOUD_PUBLISHER(nh, T, topic_name, queue_size) CREATE_PUBLISHER(nh, T, topic_name, queue_size)

#define CREATE_POINT_CLOUD_SUBSCRIPTION(nh, T, topic_name, queue_size, callback, object)                               \
  CREATE_SUBSCRIPTION(nh, T, topic_name, queue_size, callback, object)

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
