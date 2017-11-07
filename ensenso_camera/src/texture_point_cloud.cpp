#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <mutex>
#include <string>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using TexturedPointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

/**
 * Texture the given point cloud from a rectified image. This assumes that the
 * point cloud has the same format as the image and we can therefore associate
 * points with image pixels by their coordinates.
 */
TexturedPointCloud::Ptr texturePointCloudFromRectifiedImage(cv::Mat const& image,
                                                            PointCloud::ConstPtr const& pointCloud)
{
  auto texturedPointCloud = boost::make_shared<TexturedPointCloud>();

  if (static_cast<int>(pointCloud->width) != image.cols || static_cast<int>(pointCloud->height) != image.rows)
  {
    ROS_ERROR("The point cloud does not have the same format as the rectified "
              "image!");
    return texturedPointCloud;
  }

  texturedPointCloud->header.frame_id = pointCloud->header.frame_id;
  texturedPointCloud->header.stamp = pointCloud->header.stamp;

  texturedPointCloud->is_dense = pointCloud->is_dense;
  texturedPointCloud->width = pointCloud->width;
  texturedPointCloud->height = pointCloud->height;
  texturedPointCloud->points.resize(texturedPointCloud->width * texturedPointCloud->height);

  for (size_t x = 0; x < pointCloud->width; x++)
  {
    for (size_t y = 0; y < pointCloud->height; y++)
    {
      auto& point = texturedPointCloud->at(x, y);

      point.x = pointCloud->at(x, y).x;
      point.y = pointCloud->at(x, y).y;
      point.z = pointCloud->at(x, y).z;

      if (image.channels() == 3)
      {
        cv::Vec3b color = image.at<cv::Vec3b>(y, x);
        point.b = color.val[0];
        point.g = color.val[1];
        point.r = color.val[2];
      }
      else
      {
        uchar color = image.at<uchar>(y, x);
        point.r = color;
        point.g = color;
        point.b = color;
      }
    }
  }

  return texturedPointCloud;
}

class TexturingNode
{
private:
  ros::NodeHandle nodeHandle;

  std::mutex mutex;

  image_transport::Subscriber imageSubscriber;
  ros::Subscriber pointCloudSubscriber;
  ros::Publisher texturedPointCloudPublisher;

  sensor_msgs::ImageConstPtr latestImage;
  PointCloud::ConstPtr latestPointCloud;

public:
  TexturingNode()
  {
    image_transport::ImageTransport imageTransport(nodeHandle);

    imageSubscriber = imageTransport.subscribe("image", 10, &TexturingNode::onImageReceived, this);
    pointCloudSubscriber = nodeHandle.subscribe("point_cloud", 10, &TexturingNode::onPointCloudReceived, this);

    texturedPointCloudPublisher = nodeHandle.advertise<TexturedPointCloud>("textured_point_cloud", 1);
  }

private:
  void onImageReceived(sensor_msgs::ImageConstPtr const& image)
  {
    std::lock_guard<std::mutex> lock(mutex);

    latestImage = image;
  }

  void onPointCloudReceived(PointCloud::ConstPtr const& pointCloud)
  {
    std::lock_guard<std::mutex> lock(mutex);

    latestPointCloud = pointCloud;
    texture();
  }

  void texture() const
  {
    if (!latestImage.get() || !latestPointCloud.get())
      return;

    cv_bridge::CvImageConstPtr image;
    try
    {
      image = cv_bridge::toCvShare(latestImage);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    TexturedPointCloud::Ptr texturedPointCloud;
    texturedPointCloud = texturePointCloudFromRectifiedImage(image->image, latestPointCloud);

    texturedPointCloudPublisher.publish(texturedPointCloud);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "texture_point_cloud");

  TexturingNode t;
  ros::spin();

  return EXIT_SUCCESS;
}
