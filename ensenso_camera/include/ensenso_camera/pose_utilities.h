#pragma once

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // Needed for conversion from geometry_msgs to tf2::Transform
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>

#include "nxLib.h"

/**
 * Check whether the given pose is valid, that is, whether it does not contain NaNs.
 *
 * For example, such an invalid pose is produced by converting an uninitialized geometry_msgs/Pose to a tf pose.
 */
bool isValid(tf2::Transform const& pose);
bool isValid(geometry_msgs::Transform const& pose);
bool isValid(tf2::Vector3 const& vector);

/**
 * Check whether the given tf pose is an identity transformation.
 */
bool isIdentity(tf2::Transform const& pose);

/**
 * Convert the given tf pose to an NxLib transformation and write it into the given NxLib node.
 */
void writePoseToNxLib(tf2::Transform const& pose, NxLibItem const& node);

/**
 * Convert the given NxLib transformation node to a tf pose.
 */
tf2::Transform poseFromNxLib(NxLibItem const& node);

/**
 * Creates a stamped transformation from frame to child frame with given link described in the node.
 */
geometry_msgs::TransformStamped poseFromNxLib(NxLibItem const& node, std::string const& parentFrame,
                                              std::string const& childFrame, ros::Time timestamp);

/**
 * Get a tf transformation that defines the child frame at the position of the given pose.
 */
geometry_msgs::TransformStamped transformFromPose(geometry_msgs::PoseStamped const& pose,
                                                  std::string const& childFrame);

/**
 * Get a geometry pose from a transform, where pose has the same rotation and translation as the transform.
 */
geometry_msgs::PoseStamped stampedPoseFromTransform(geometry_msgs::TransformStamped const& transform);
geometry_msgs::Pose poseFromTransform(tf2::Transform const& transform);
geometry_msgs::TransformStamped fromTfTransform(tf2::Transform const& transform, std::string parentFrame,
                                                std::string childFrame, ros::Time timestamp);

/**
 * Creates a tf2::Transform out of pose or transform message type.
 */
tf2::Transform fromMsg(geometry_msgs::Transform const& transform);
tf2::Transform fromMsg(geometry_msgs::Pose const& pose);
tf2::Transform fromStampedMessage(geometry_msgs::TransformStamped const& transform);
tf2::Transform fromStampedMessage(geometry_msgs::PoseStamped const& pose);

tf2::Transform getLatestTransform(tf2_ros::Buffer const& tfBuffer, std::string const& cameraFrame,
                                  std::string const& targetFrame);
