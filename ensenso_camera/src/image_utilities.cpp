#include "ensenso_camera/image_utilities.h"

#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

double const NXLIB_TIMESTAMP_OFFSET = 11644473600;

std::string imageEncoding(bool isFloat, int channels, int bytesPerElement)
{
  if (isFloat)
  {
    switch (channels)
    {
      case 1:
        return (bytesPerElement == 4) ? sensor_msgs::image_encodings::TYPE_32FC1 :
                                        sensor_msgs::image_encodings::TYPE_64FC1;
      case 2:
        return (bytesPerElement == 4) ? sensor_msgs::image_encodings::TYPE_32FC2 :
                                        sensor_msgs::image_encodings::TYPE_64FC2;
      case 3:
        return (bytesPerElement == 4) ? sensor_msgs::image_encodings::TYPE_32FC3 :
                                        sensor_msgs::image_encodings::TYPE_64FC3;
      case 4:
        return (bytesPerElement == 4) ? sensor_msgs::image_encodings::TYPE_32FC4 :
                                        sensor_msgs::image_encodings::TYPE_64FC4;
    }
  }
  else
  {
    switch (channels)
    {
      case 1:
        switch (bytesPerElement)
        {
          case 1:
            return sensor_msgs::image_encodings::TYPE_8UC1;
          case 2:
            return sensor_msgs::image_encodings::TYPE_16UC1;
          case 4:
            return sensor_msgs::image_encodings::TYPE_32SC1;
        }
        break;
      case 2:
        switch (bytesPerElement)
        {
          case 1:
            return sensor_msgs::image_encodings::TYPE_8UC2;
          case 2:
            return sensor_msgs::image_encodings::TYPE_16UC2;
          case 4:
            return sensor_msgs::image_encodings::TYPE_32SC2;
        }
        break;
      case 3:
        switch (bytesPerElement)
        {
          case 1:
            return sensor_msgs::image_encodings::TYPE_8UC3;
          case 2:
            return sensor_msgs::image_encodings::TYPE_16UC3;
          case 4:
            return sensor_msgs::image_encodings::TYPE_32SC3;
        }
        break;
      case 4:
        switch (bytesPerElement)
        {
          case 1:
            return sensor_msgs::image_encodings::TYPE_8UC4;
          case 2:
            return sensor_msgs::image_encodings::TYPE_16UC4;
          case 4:
            return sensor_msgs::image_encodings::TYPE_32SC4;
        }
        break;
    }
  }

  ROS_ERROR("Invalid image encoding in binary NxLib node.");
  return "";
}

sensor_msgs::ImagePtr imageFromNxLibNode(NxLibItem const& node, std::string const& frame)
{
  auto image = boost::make_shared<sensor_msgs::Image>();

  bool isFloat;
  int width, height, channels, bytesPerElement;
  double timestamp;
  node.getBinaryDataInfo(&width, &height, &channels, &bytesPerElement, &isFloat, &timestamp);

  image->header.stamp.fromSec(timestamp - NXLIB_TIMESTAMP_OFFSET);
  image->header.frame_id = frame;
  image->width = width;
  image->height = height;
  image->step = width * channels * bytesPerElement;
  image->encoding = imageEncoding(isFloat, channels, bytesPerElement);

  node.getBinaryData(image->data, 0);

  return image;
}

std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>
imagePairFromNxLibNode(const NxLibItem& node, std::string const& frame)
{
  auto leftImage = imageFromNxLibNode(node[itmLeft], frame);
  auto rightImage = imageFromNxLibNode(node[itmRight], frame);

  return { leftImage, rightImage };
}

std::vector<std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>
imagesFromNxLibNode(const NxLibItem& node, std::string const& frame)
{
  std::vector<std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>> result;

  if (node.isArray())
  {
    for (int i = 0; i < node.count(); i++)
    {
      result.push_back(imagePairFromNxLibNode(node[i], frame));
    }
  }
  else
  {
    result.push_back(imagePairFromNxLibNode(node, frame));
  }

  return result;
}

ros::Time timestampFromNxLibNode(const NxLibItem& node)
{
  double timestamp;
  node.getBinaryDataInfo(0, 0, 0, 0, 0, &timestamp);

  return ros::Time(timestamp - NXLIB_TIMESTAMP_OFFSET);
}
