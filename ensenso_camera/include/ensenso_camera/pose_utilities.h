#pragma once

#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <string>

#include "nxLib.h"

/**
 * Check whether the given pose is valid, that is, whether it does not contain
 * NaNs.
 *
 * For example, such an invalid pose is produced by converting an uninitialized
 * geometry_msgs/Pose to a TF pose.
 */
bool poseIsValid(tf::Pose const& transform);

/**
 * Convert the given TF pose to an NxLib transformation and write it into
 * the given NxLib node.
 */
void writePoseToNxLib(tf::Pose const& pose, NxLibItem const& node);

/**
 * Convert the given NxLib transformation node to a TF pose.
 */
tf::Pose poseFromNxLib(NxLibItem const& node);

tf::Stamped<tf::Pose> poseFromNxLib(NxLibItem const& node, ros::Time const& timestamp, std::string const& frame);

/**
 * Get a TF transformation that defines the child frame at the position of the
 * given pose.
 */
tf::StampedTransform transformFromPose(geometry_msgs::PoseStamped const& pose, std::string const& childFrame);

/**
 * @brief      Publish pose to tf
 *
 * @param[in]  virtualCamPose         The virtual camera pose
 * @param[in]  baseFrame              The base frame
 * @param[in]  targetFrame            The target frame
 * @param[in]  static_tf_broadcaster  The static tf broadcaster
 */
void publishCameraPose(tf::StampedTransform virtualCamPose, std::string baseFrame, std::string targetFrame, tf2_ros::StaticTransformBroadcaster static_tf_broadcaster);


/**
 * @brief      Calculates the leveled camera pose. The returned pose has its xy plane paralled to that of the world
 *
 * @param[in]  originalPose  The original pose
 *
 * @return     The leveled camera pose.
 */
tf::StampedTransform computeLeveledCameraPose(tf::StampedTransform originalPose);
