#pragma once

#include <utility>
#include <string>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "nxLib.h"

using Image = sensor_msgs::Image;
using ImagePtr = sensor_msgs::ImagePtr;
using ImagePtrPair = std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>;

/**
 * Convert the given binary NxLib node to a ROS image message.
 */
ImagePtr imageFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera);

/**
 * Convert the given NxLib node to a set of image pairs.
 *
 * This function supports different types of image nodes. When the node is an array, it will create an image pair for
 * each entry of the array (this is the case when FlexView is enabled). Otherwise it creates an image pair from the left
 * and right child nodes (as it is the case when FlexView is disabled or not supported by a camera).
 */
std::vector<ImagePtrPair> imagePairsFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera);

/**
 * Convert the given NxLib node to a set of images.
 *
 * This function supports different types of image nodes. When the node is an array, it will create an image for each
 * entry of the array. Otherwise it creates an image from the child node.
 */
std::vector<ImagePtr> imagesFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera);

/**
 * Get the timestamp from an NxLib image node.
 */
ros::Time timestampFromNxLibNode(NxLibItem const& node);

/**
 * Get the z-channel from the calculated point cloud and transform it into a sensor_msgs/Image depth image defined in
 * REP 118 - depth images. The Image has a canonical format (z-values in Meters, float 1-channel image (TYPE_32FC1)).
 */
ImagePtr depthImageFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera);

/**
 * Gets the corresponding distortion parameters from the Item.
 */
void fillDistortionParamsFromNxLib(NxLibItem const& distortionItem, sensor_msgs::CameraInfoPtr const& info);
