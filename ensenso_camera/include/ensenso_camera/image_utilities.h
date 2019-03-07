#pragma once

#include <utility>
#include <string>
#include <vector>

#include <sensor_msgs/Image.h>

#include "nxLib.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

/**
 * Convert the given binary NxLib node to a ROS image message.
 */
sensor_msgs::ImagePtr imageFromNxLibNode(NxLibItem const& node, std::string const& frame);

/**
 * Convert the given NxLib node to a set of image pairs.
 *
 * This function supports different types of image nodes. When the node is an
 * array, it will create an image pair for each entry of the array (this is the
 * case when FlexView is enabled). Otherwise it creates an image pair from the
 * left and right child nodes (as it is the case when FlexView is disabled or
 * not supported by a camera).
 */
std::vector<std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>
imagesFromNxLibNode(NxLibItem const& node, std::string const& frame);

/**
 * Get the timestamp from an NxLib image node.
 */
ros::Time timestampFromNxLibNode(NxLibItem const& node);

/**
 * Convert the given binary NxLib node to an opencv Mat.
 */
void imageFromNxLibNodeToOpencvMat(cv::Mat& cvMat, NxLibItem const& node);