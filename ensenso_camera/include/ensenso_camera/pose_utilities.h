#pragma once

#include <tf/transform_datatypes.h>
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
