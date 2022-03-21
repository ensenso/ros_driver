// Do not change the order of this block. Otherwise getBinaryData with CV::mat overload will not be recognized
#include <cv_bridge/cv_bridge.h>
#include "ensenso_camera/image_utilities.h"

#include "ensenso_camera/conversion.h"

#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

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
            return sensor_msgs::image_encodings::MONO8;
          case 2:
            return sensor_msgs::image_encodings::MONO16;
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
            return sensor_msgs::image_encodings::RGB8;
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

ImagePtr imageFromNxLibNode(NxLibItem const& node, std::string const& frame)
{
  auto image = boost::make_shared<Image>();

  bool isFloat;
  int width, height, channels, bytesPerElement;
  double timestamp;
  node.getBinaryDataInfo(&width, &height, &channels, &bytesPerElement, &isFloat, &timestamp);

  image->header.stamp.fromSec(ensenso_conversion::nxLibToRosTimestamp(timestamp));
  image->header.frame_id = frame;
  image->width = width;
  image->height = height;
  image->step = width * channels * bytesPerElement;
  image->encoding = imageEncoding(isFloat, channels, bytesPerElement);

  node.getBinaryData(image->data, 0);

  return image;
}

ImagePtrPair imagePairFromNxLibNode(NxLibItem const& node, std::string const& frame)
{
  ImagePtr leftImage;
  ImagePtr rightImage;

  if (node[itmLeft].exists() && node[itmRight].exists())
  {
    leftImage = imageFromNxLibNode(node[itmLeft], frame);
    rightImage = imageFromNxLibNode(node[itmRight], frame);
  }
  else
  {
    // Create a dummy pair with an empty right image in case of an S-series camera.
    leftImage = imageFromNxLibNode(node, frame);
    rightImage = nullptr;
  }

  return { leftImage, rightImage };
}

std::vector<ImagePtrPair> imagePairsFromNxLibNode(NxLibItem const& node, std::string const& frame)
{
  std::vector<ImagePtrPair> result;

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

std::vector<ImagePtr> imagesFromNxLibNode(NxLibItem const& node, std::string const& frame)
{
  std::vector<ImagePtr> result;

  if (node.isArray())
  {
    for (int i = 0; i < node.count(); i++)
    {
      result.push_back(imageFromNxLibNode(node[i], frame));
    }
  }
  else
  {
    result.push_back(imageFromNxLibNode(node, frame));
  }

  return result;
}

ros::Time timestampFromNxLibNode(NxLibItem const& node)
{
  double timestamp;
  node.getBinaryDataInfo(0, 0, 0, 0, 0, &timestamp);

  return ros::Time(ensenso_conversion::nxLibToRosTimestamp(timestamp));
}

ImagePtr depthImageFromNxLibNode(NxLibItem const& node, std::string const& frame)
{
  double timestamp;
  cv::Mat pointMap;
  node.getBinaryData(pointMap, &timestamp);

  cv::Mat depthImage;
  cv::extractChannel(pointMap, depthImage, 2);

  // Convert units from millimeters to meters.
  depthImage /= 1000.0;

  // Convert cv mat to ros image.
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp.fromSec(ensenso_conversion::nxLibToRosTimestamp(timestamp));
  out_msg.header.frame_id = frame;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depthImage;

  return out_msg.toImageMsg();
}

void fillDistortionParamsFromNxLib(NxLibItem const& distortionItem, sensor_msgs::CameraInfoPtr const& info)
{
  std::vector<double> distParams(5, 0.);
  bool isPlumbModel = info->distortion_model == sensor_msgs::distortion_models::PLUMB_BOB;
  if (!isPlumbModel)
  {
    info->D = distParams;
    return;
  }

  auto getNxLibValue = [](NxLibItem const& itemToCheck) -> double {
    double value = 0.;
    try
    {
      value = itemToCheck.asDouble();
    }
    catch (...)
    {
      ROS_WARN("The distortion parameter %s does not exist. Using value 0.0 instead.", itemToCheck.path.c_str());
    }
    return value;
  };

  if (distortionItem.isObject())
  {
    distParams[0] = getNxLibValue(distortionItem[itmK1]);
    distParams[1] = getNxLibValue(distortionItem[itmK2]);
    distParams[2] = getNxLibValue(distortionItem[itmT1]);
    distParams[3] = getNxLibValue(distortionItem[itmT2]);
    distParams[4] = getNxLibValue(distortionItem[itmK3]);
  }
  else if (distortionItem.isArray())
  {
    for (int i = 0; i < 5; i++)
    {
      info->D.push_back(getNxLibValue(distortionItem[i]));
    }
  }
  info->D = distParams;
}
