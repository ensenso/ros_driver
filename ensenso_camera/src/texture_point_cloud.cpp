#include "ensenso_camera/point_cloud_utilities.h"

#include "ensenso_camera/ros2_core.h"
#include "ensenso_camera/ros2_image_transport.h"
#include "ensenso_camera/ros2_logging.h"
#include "ensenso_camera/ros2_namespace.h"
#include "ensenso_camera/ros2_node.h"
#include "ensenso_camera/ros2_node_handle.h"
#include "ensenso_camera/ros2_node_wrapper.h"
#include "ensenso_camera/ros2_pcl.h"

// Make sure cv_bridge is imported before image_transport. Otherwise
// getBinaryData with CV::mat overload will not be recognized.
#include <cv_bridge/cv_bridge.h>

#ifdef ROS2
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#else
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#endif

#include <mutex>
#include <string>

#define NODE_CLASS_NAME TexturePointCloudNode
#define NODE_NAME "texture_point_cloud"

USING_MSG(sensor_msgs, Image)

SINGLE_NODE_CLASS(NODE_CLASS_NAME)
{
private:
  ensenso::ros::NodeHandle nh;

  std::mutex mutex;

  image_transport::Subscriber imageSubscriber;
  POINT_CLOUD_SUBSCRIPTION(ensenso::PointCloud) pointCloudSubscription;
  POINT_CLOUD_PUBLISHER(ensenso::PointCloudColored) texturedPointCloudPublisher;

  sensor_msgs::msg::ImageConstPtr latestImage;
  std::shared_ptr<ensenso::PointCloud> latestPointCloud;

public:
  SINGLE_NODE_CTOR(NODE_CLASS_NAME, NODE_NAME)
  {
    CREATE_NODE_HANDLE(nh);

    IMAGE_TRANSPORT_INIT(nh);
    imageSubscriber = IMAGE_TRANSPORT_CREATE_SUBSCRIPTION(nh, "image", &NODE_CLASS_NAME::onImageReceived, this);

    pointCloudSubscription = CREATE_POINT_CLOUD_SUBSCRIPTION(nh, ensenso::PointCloud, "point_cloud", 10,
                                                             &NODE_CLASS_NAME::onPointCloudReceived, this);

    texturedPointCloudPublisher =
        CREATE_POINT_CLOUD_PUBLISHER(nh, ensenso::PointCloudColored, "textured_point_cloud", 1);

    latestPointCloud = std::make_shared<ensenso::PointCloud>();
  }

private:
  void onImageReceived(sensor_msgs::msg::ImageConstPtr const& image)
  {
    std::lock_guard<std::mutex> lock(mutex);

    latestImage = image;
  }

  void POINT_CLOUD_SUBSCRIPTION_CALLBACK(onPointCloudReceived, ensenso::PointCloud, pointCloud)
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

  std::unique_ptr<ensenso::PointCloudColored> texturePointCloudFromRectifiedImage(cv::Mat const& image)
  {
    auto texturedPointCloud = ensenso::std::make_unique<ensenso::PointCloudColored>();

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
