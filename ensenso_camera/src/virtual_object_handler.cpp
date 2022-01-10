#include <ensenso_camera/virtual_object_handler.h>

#include <ensenso_camera/pose_utilities.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>

#include <visualization_msgs/MarkerArray.h>

#include <nxLib.h>


using namespace ensenso_camera;

namespace {
    std::string readFile(const std::string &filename)
    {
        std::ifstream file{ filename };
        if (!file.is_open() || !file.rdbuf()) {
            throw std::runtime_error("Unable to read objects file");
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    /// Class responsible for publishing NxLib virtual objects as ROS visualization_msgs::MarkerArray
    struct VirtualObjectMarkerPublisher
    {
      VirtualObjectMarkerPublisher(const std::string &topic, double publishRate, const NxLibItem& objects,
                                   const std::string &frame, std::atomic_bool& stop_)
        : rate(publishRate), stop(stop_)
      {
        // Create output topic
        ros::NodeHandle node;
        publisher = node.advertise<visualization_msgs::MarkerArray>(topic, 1);

        // Collect markers
        for (int i = 0; i < objects.count(); ++i)
        {
          auto& object = objects[i];
          auto type = object[itmType];

          visualization_msgs::Marker marker;
          marker.ns = ros::this_node::getName();
          marker.id = i;
          marker.header.stamp = ros::Time::now();
          marker.header.frame_id = frame;
          marker.action = visualization_msgs::Marker::MODIFY;  // Note: ADD = MODIFY

          // Set color
          const auto color = object[itmLighting][itmColor];
          if (color.exists() && color.count() == 3)
          {
            marker.color.r = color[0].asDouble();
            marker.color.g = color[1].asDouble();
            marker.color.b = color[2].asDouble();
            marker.color.a = 1.0;
          }
          else
          {
            // Default color (gray)
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 1.0;
          }

          // Set pose
          marker.pose = poseFromTransform(poseFromNxLib(object[itmLink]));

          // Set type and deal with object-specific properties
          if (type.asString() == valSphere)
          {
            marker.type = visualization_msgs::Marker::SPHERE;

            // Sphere scale is diameter in meters
            marker.scale.x = object[itmRadius].asDouble() * 2.0 / 1000.0;
            marker.scale.y = marker.scale.x;
            marker.scale.z = marker.scale.x;
          }
          else if (type.asString() == valCylinder)
          {
            marker.type = visualization_msgs::Marker::CYLINDER;

            /// Cylinder scale x/y is diameter in meters, z is height in meters.
            marker.scale.x = object[itmRadius].asDouble() * 2.0 / 1000.0;
            marker.scale.y = marker.scale.x;
            marker.scale.z = object[itmHeight].asDouble() / 1000.0;

            /// Cylinders in NxLib start at the base.
            /// Cylinders in ROS start at the centroid.
            // marker.pose.position.z += marker.scale.z / 2.0;
          }
          else if (type.asString() == valCube)
          {
            marker.type = visualization_msgs::Marker::CUBE;

            marker.scale.x = object[itmWidth].asDouble() / 1000.0;
            marker.scale.y = object[itmWidth].asDouble() / 1000.0;
            marker.scale.z = object[itmWidth].asDouble() / 1000.0;
          }
          else if (type.asString() == valCuboid)
          {
            marker.type = visualization_msgs::Marker::CUBE;

            marker.scale.x = object[itmWidth].asDouble() / 1000.0;
            marker.scale.z = object[itmHeight].asDouble() / 1000.0;
            marker.scale.y = object[itmDepth].asDouble() / 1000.0;
          }
          else
          {
            // Unsupported type, skip it
            continue;
          }

          markers.markers.push_back(marker);
        }
      }

      void run()
      {
        while (ros::ok() && !stop)
        {
          publisher.publish(markers);
          rate.sleep();
        }
      }

      ros::Publisher publisher;
      ros::Rate rate;
      visualization_msgs::MarkerArray markers;
      std::atomic_bool& stop;
    };
}

VirtualObjectHandler::VirtualObjectHandler(const std::string &filename, const std::string &objectsFrame, const std::string &linkFrame,
                                           const std::string &markerTopic, double markerPublishRate) :
    objectsFrame(objectsFrame),
    linkFrame(linkFrame)
{
    nxLibInitialize(false);

    // Read the file contents and assign it to the objects tag
    auto objects = NxLibItem{ itmObjects };
    objects.setJson(readFile(filename));

    // Get the original poses from file
    for (int i = 0; i < objects.count(); ++i) {
        originalPoses.push_back(poseFromNxLib(objects[i][itmLink]));
    }

    // Create publisher thread
    if (!markerTopic.empty()) {
      markerThread = std::thread([=](){
        VirtualObjectMarkerPublisher publisher{markerTopic, markerPublishRate, objects, objectsFrame, stopMarkerThread};
        publisher.run();
      });
    }
}

VirtualObjectHandler::~VirtualObjectHandler() {
  stopMarkerThread = true;
}

void VirtualObjectHandler::updateObjectLinks() {
    // Exit early if we have no work to do
    if (originalPoses.empty())
    {
        return;
    }

    // Find transform from the frame in which the objects were defined to the current optical frame
    tf2::Transform cameraPose;
    try
    {
        cameraPose = fromMsg(tfBuffer.lookupTransform(linkFrame, objectsFrame, ros::Time(0)).transform);
    }
    catch (const tf2::TransformException& e)
    {
        ROS_WARN("Could not look up the virtual object pose due to the TF error: %s", e.what());
        return;
    }

    // Apply the transform to all of the original poses
    for (size_t i = 0; i < originalPoses.size(); ++i)
    {
        tf2::Transform objectPose = cameraPose * originalPoses[i];
        NxLibItem objectLink = NxLibItem{ itmObjects }[static_cast<int>(i)][itmLink];
        writePoseToNxLib(objectPose.inverse(), objectLink);
    }
}
