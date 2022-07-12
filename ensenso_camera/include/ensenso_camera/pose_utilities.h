#pragma once

#include "ensenso_camera/ros2_namespace.h"
#include "ensenso_camera/ros2_time.h"

#ifdef ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#else
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#endif

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <string>

#include "nxLib.h"

USING_MSG(geometry_msgs, Pose)
USING_MSG(geometry_msgs, PoseStamped)
USING_MSG(geometry_msgs, Transform)
USING_MSG(geometry_msgs, TransformStamped)

using Transform = tf2::Transform;
using TransformMsg = geometry_msgs::msg::Transform;
using StampedTransformMsg = geometry_msgs::msg::TransformStamped;
using PoseMsg = geometry_msgs::msg::Pose;
using StampedPoseMsg = geometry_msgs::msg::PoseStamped;

/**
 * Check whether the given transform is valid, i.e. not containing NaNs.
 *
 * An invalid transform is e.g. produced by converting an uninitialized transform msg to a tf transform.
 */
bool isValid(Transform const& transform);
bool isValid(TransformMsg const& transform);

/**
 * Check whether the given tf transform is an identity transformation.
 */
bool isIdentity(Transform const& transform);

/**
 * Convert the given tf transform to an NxLib transformation and write it into the given NxLib node.
 */
void writeTransformToNxLib(Transform const& transform, NxLibItem const& node);

/**
 * Convert the given NxLib transformation node to a tf transform.
 */
Transform transformFromNxLib(NxLibItem const& node);

/**
 * Create a stamped pose msg from the given link described in the node.
 */
StampedPoseMsg stampedPoseFromNxLib(NxLibItem const& node, std::string const& parentFrame,
                                    ensenso::ros::Time timestamp);

/**
 * Convert a stamped pose msg to a stamped transform msg that defines the child frame at the position of the given pose.
 */
StampedTransformMsg transformFromPose(StampedPoseMsg const& pose, std::string const& childFrame);

/**
 * Convert a stamped transform msg to a stamped pose msg, where the pose has the same rotation and translation as the
 * transform.
 */
StampedPoseMsg poseFromTransform(StampedTransformMsg const& transform);

/**
 * Convert a tf transform to a pose msg.
 */
PoseMsg poseFromTransform(Transform const& transform);

/**
 * Create a stamped geometry transform from a tf transform in combination with parent and child frame.
 */
StampedTransformMsg fromTf(Transform const& transform, std::string parentFrame, std::string childFrame,
                           ensenso::ros::Time timestamp);

/**
 * Create a tf transform from a transform msg.
 */
Transform fromMsg(TransformMsg const& transform);

/**
 * Create a tf transform from a pose msg.
 */
Transform fromMsg(PoseMsg const& pose);

/**
 * Create a tf transform from a stamped transform msg.
 */
Transform fromMsg(StampedTransformMsg const& transform);

/**
 * Create a tf transform from a stamped pose msg.
 */
Transform fromMsg(StampedPoseMsg const& pose);

/**
 * Return the latest transform from the given buffer. Returns an uninitialized tf transform if the lookup throws and
 * prints a warning.
 */
Transform getLatestTransform(tf2_ros::Buffer const& tfBuffer, std::string const& cameraFrame,
                             std::string const& targetFrame);

/**
 * The tf2::convert method is designed for converting homogeneous datatypes between representations in different
 * libraries, e.g. tf2::Transform and geometry_msgs::Transform. In order to simplify our conversion methods, it is handy
 * to have a set of functions for converting inhomogenous datatypes of the geometry_msgs package.
 *
 * Source: https://answers.ros.org/question/206962/tf2-convert-transform-to-msg/
 */
namespace tf2
{
void convertMsg(TransformMsg const& transform, PoseMsg& pose);
void convertMsg(PoseMsg const& pose, TransformMsg& transform);
void convertMsg(StampedTransformMsg const& transform, StampedPoseMsg& pose);
void convertMsg(StampedPoseMsg const& pose, StampedTransformMsg& transform);
}  // namespace tf2
