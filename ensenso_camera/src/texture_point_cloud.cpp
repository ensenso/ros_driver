#include "ensenso_camera/point_cloud_utilities.h"

// Make sure cv_bridge is imported before image_transport. Otherwise
// getBinaryData with CV::mat overload will not be recognized.
#include "ensenso_camera/ros2/cv_bridge/cv_bridge.h"

#include "ensenso_camera/ros2/core.h"
#include "ensenso_camera/ros2/image_transport.h"
#include "ensenso_camera/ros2/logging.h"
#include "ensenso_camera/ros2/namespace.h"
#include "ensenso_camera/ros2/node.h"
#include "ensenso_camera/ros2/node_handle.h"
#include "ensenso_camera/ros2/node_wrapper.h"
#include "ensenso_camera/ros2/pcl.h"

#include "ensenso_camera/ros2/sensor_msgs/image.h"

#include <mutex>
#include <string>

#define NODE_CLASS_NAME TexturePointCloudNode
#define NODE_NAME "texture_point_cloud"

SINGLE_NODE_CLASS(NODE_CLASS_NAME)
{
private:
  ensenso::ros::NodeHandle nh;

  std::mutex mutex;

  image_transport::Subscriber imageSubscriber;
  PointCloudSubscription<ensenso::pcl::PointCloud> pointCloudSubscription;
  PointCloudPublisher<ensenso::pcl::PointCloudColored> texturedPointCloudPublisher;

  sensor_msgs::msg::ImageConstPtr latestImage;
  std::shared_ptr<ensenso::pcl::PointCloud> latestPointCloud;

public:
  SINGLE_NODE_CTOR(NODE_CLASS_NAME, NODE_NAME)
  {
    CREATE_NODE_HANDLE(nh);

    imageSubscriber =
        ensenso::image_transport::create_subscription(nh, "image", &NODE_CLASS_NAME::onImageReceived, this);

    pointCloudSubscription = ensenso::pcl::create_subscription<ensenso::pcl::PointCloud>(
        nh, "point_cloud", 10, &NODE_CLASS_NAME::onPointCloudReceived, this);

    texturedPointCloudPublisher =
        ensenso::pcl::create_publisher<ensenso::pcl::PointCloudColored>(nh, "textured_point_cloud", 1);

    latestPointCloud = std::make_shared<ensenso::pcl::PointCloud>();
  }

private:
  void onImageReceived(sensor_msgs::msg::ImageConstPtr const& image)
  {
    std::lock_guard<std::mutex> lock(mutex);

    latestImage = image;
  }

  void POINT_CLOUD_SUBSCRIPTION_CALLBACK(onPointCloudReceived, ensenso::pcl::PointCloud, pointCloud)
  {
    std::lock_guard<std::mutex> lock(mutex);

    STORE_POINT_CLOUD(pointCloud, latestPointCloud);

    texture();
  }

  void texture()
  {
    if (!latestImage.get() || !latestPointCloud.get())
    {
      return;
    }

    cv_bridge::CvImageConstPtr image;
    try
    {
      image = cv_bridge::toCvShare(latestImage);
    }
    catch (cv_bridge::Exception& e)
    {
      ENSENSO_ERROR(nh, "cv_bridge exception: %s", e.what());
    }

    auto texturedPointCloud = texturePointCloudFromRectifiedImage(image->image);

    publishPointCloud(texturedPointCloudPublisher, std::move(texturedPointCloud));
  }

  std::unique_ptr<ensenso::pcl::PointCloudColored> texturePointCloudFromRectifiedImage(cv::Mat const& image)
  {
    auto texturedPointCloud = ensenso::std::make_unique<ensenso::pcl::PointCloudColored>();

    if (static_cast<int>(latestPointCloud->width) != image.cols ||
        static_cast<int>(latestPointCloud->height) != image.rows)
    {
      ENSENSO_ERROR(nh, "The point cloud does not have the same format as the rectified image!");
      return texturedPointCloud;
    }

    texturedPointCloud->header.frame_id = latestPointCloud->header.frame_id;
    texturedPointCloud->header.stamp = latestPointCloud->header.stamp;

    texturedPointCloud->is_dense = latestPointCloud->is_dense;
    texturedPointCloud->width = latestPointCloud->width;
    texturedPointCloud->height = latestPointCloud->height;
    texturedPointCloud->points.resize(texturedPointCloud->width * texturedPointCloud->height);

    for (size_t x = 0; x < latestPointCloud->width; x++)
    {
      for (size_t y = 0; y < latestPointCloud->height; y++)
      {
        auto& point = texturedPointCloud->at(x, y);

        point.x = latestPointCloud->at(x, y).x;
        point.y = latestPointCloud->at(x, y).y;
        point.z = latestPointCloud->at(x, y).z;

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
};

int main(int argc, char** argv)
{
  SINGLE_NODE_WRAPPER(NODE_CLASS_NAME, NODE_NAME)

  return 0;
}
