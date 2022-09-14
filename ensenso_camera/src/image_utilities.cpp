// Do not change the order of this block. Otherwise getBinaryData with CV::mat overload will not be recognized
#include <cv_bridge/cv_bridge.h>
#include "ensenso_camera/image_utilities.h"

#include "ensenso_camera/conversion.h"

#include "ensenso_camera/ros2/core.h"
#include "ensenso_camera/ros2/logging.h"

#include "ensenso_camera/ros2/sensor_msgs/camera_info.h"
#include "ensenso_camera/ros2/sensor_msgs/distortion_models.h"
#include "ensenso_camera/ros2/sensor_msgs/image_encodings.h"

#include <string>
#include <utility>
#include <vector>

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

  ENSENSO_ERROR("Invalid image encoding in binary NxLib node.");
  return "";
}

ImagePtr imageFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera)
{
  auto image = ensenso::std::make_shared<Image>();

  bool isFloat;
  int width, height, channels, bytesPerElement;
  double nxLibTimestamp;
  node.getBinaryDataInfo(&width, &height, &channels, &bytesPerElement, &isFloat, &nxLibTimestamp);

  double timestamp = ensenso_conversion::nxLibToRosTimestamp(nxLibTimestamp, isFileCamera);

  image->header.stamp = ensenso::ros::timeFromSeconds(timestamp);
  image->header.frame_id = frame;
  image->width = width;
  image->height = height;
  image->step = width * channels * bytesPerElement;
  image->encoding = imageEncoding(isFloat, channels, bytesPerElement);

  node.getBinaryData(image->data, 0);

  return image;
}

ImagePtrPair imagePairFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera)
{
  ImagePtr leftImage;
  ImagePtr rightImage;

  if (node[itmLeft].exists() && node[itmRight].exists())
  {
    leftImage = imageFromNxLibNode(node[itmLeft], frame, isFileCamera);
    rightImage = imageFromNxLibNode(node[itmRight], frame, isFileCamera);
  }
  else
  {
    // Create a dummy pair with an empty right image in case of an S-series camera.
    leftImage = imageFromNxLibNode(node, frame, isFileCamera);
    rightImage = nullptr;
  }

  return { leftImage, rightImage };
}

std::vector<ImagePtrPair> imagePairsFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera)
{
  std::vector<ImagePtrPair> result;

  if (node.isArray())
  {
    for (int i = 0; i < node.count(); i++)
    {
      result.push_back(imagePairFromNxLibNode(node[i], frame, isFileCamera));
    }
  }
  else
  {
    result.push_back(imagePairFromNxLibNode(node, frame, isFileCamera));
  }

  return result;
}

std::vector<ImagePtr> imagesFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera)
{
  std::vector<ImagePtr> result;

  if (node.isArray())
  {
    for (int i = 0; i < node.count(); i++)
    {
      result.push_back(imageFromNxLibNode(node[i], frame, isFileCamera));
    }
  }
  else
  {
    result.push_back(imageFromNxLibNode(node, frame, isFileCamera));
  }

  return result;
}

ensenso::ros::Time timestampFromNxLibNode(NxLibItem const& node)
{
  double nxLibTimestamp;
  node.getBinaryDataInfo(0, 0, 0, 0, 0, &nxLibTimestamp);

  return ensenso::ros::timeFromSeconds(ensenso_conversion::nxLibToRosTimestamp(nxLibTimestamp));
}

ImagePtr depthImageFromNxLibNode(NxLibItem const& node, std::string const& frame, bool isFileCamera)
{
  double nxLibTimestamp;
  cv::Mat pointMap;
  node.getBinaryData(pointMap, &nxLibTimestamp);

  cv::Mat depthImage;
  cv::extractChannel(pointMap, depthImage, 2);

  // Convert units from millimeters to meters.
  depthImage /= 1000.0;

  double timestamp = ensenso_conversion::nxLibToRosTimestamp(nxLibTimestamp, isFileCamera);

  // Convert cv mat to ros image.
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ensenso::ros::timeFromSeconds(timestamp);
  out_msg.header.frame_id = frame;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depthImage;

  return out_msg.toImageMsg();
}

void fillDistortionParamsFromNxLib(NxLibItem const& distortionItem, sensor_msgs::msg::CameraInfoPtr const& info)
{
  std::vector<double> distParams(5, 0.);

  auto getNxLibValue = [](NxLibItem const& itemToCheck) -> double {
    double value = 0.;
    try
    {
      value = itemToCheck.asDouble();
    }
    catch (...)
    {
      ENSENSO_WARN("The distortion parameter %s does not exist. Using value 0.0 instead.", itemToCheck.path.c_str());
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
      distParams[i] = getNxLibValue(distortionItem[i]);
    }
  }

  GET_D_MATRIX(info) = distParams;
}
