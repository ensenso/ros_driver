#include "ensenso_camera/virtual_object_handler.h"

#include "ensenso_camera/pose_utilities.h"

#include "ensenso_camera/ros2/core.h"
#include "ensenso_camera/ros2/logging.h"
#include "ensenso_camera/ros2/tf2.h"
#include "ensenso_camera/ros2/time.h"

#include "ensenso_camera/ros2/visualization_msgs/marker.h"
#include "ensenso_camera/ros2/visualization_msgs/marker_array.h"

#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "nxLib.h"

using namespace ensenso_camera;

namespace
{
    std::string readFile(const std::string& filename)
    {
        std::ifstream file{ filename };
        if (!file.is_open() || !file.rdbuf())
        {
            throw std::runtime_error("Unable to read objects file");
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

/// Class responsible for publishing NxLib virtual objects as ROS visualization_msgs::MarkerArray
    struct VirtualObjectMarkerPublisher
    {
        VirtualObjectMarkerPublisher(ensenso::ros::NodeHandle _nh, const std::string& topic, double publishRate,
                                     const NxLibItem& objects, const std::string& frame, std::atomic_bool& stop_)
                : nh(std::move(_nh)), rate(publishRate), stop(stop_)
        {
            // Create output topic
            publisher = ensenso::ros::create_publisher<visualization_msgs::msg::MarkerArray>(nh, topic, 1);

            // Collect markers
            for (int i = 0; i < objects.count(); ++i)
            {
                auto& object = objects[i];

                visualization_msgs::msg::Marker marker;

                marker.ns = ensenso::ros::get_node_name(nh);
                marker.id = i;
                marker.header.stamp = ensenso::ros::Time(0);
                marker.header.frame_id = object[itmLink][itmTarget].asString();
                marker.action = visualization_msgs::msg::Marker::MODIFY;  // Note: ADD = MODIFY

                // Set color
                const auto color = object[itmLighting][itmColor];
                if (color.exists() && color.count() == 3)
                {
                    marker.color.r = static_cast<float>(color[0].asDouble());
                    marker.color.g = static_cast<float>(color[1].asDouble());
                    marker.color.b = static_cast<float>(color[2].asDouble());
                    marker.color.a = 1.0f;
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
                marker.pose = poseFromTransform(transformFromNxLib(object[itmLink]));

                auto type = object[itmType];
                auto filename = object[itmFilename];

                // Set type and deal with object-specific properties
                if (filename.exists())
                {
                    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                    marker.mesh_resource = "file://" + filename.asString();

                    // Scale from mm to m
                    marker.scale.x = 1.0 / 1000.0;
                    marker.scale.y = 1.0 / 1000.0;
                    marker.scale.z = 1.0 / 1000.0;
                }
                else if (type.asString() == valSphere)
                {
                    marker.type = visualization_msgs::msg::Marker::SPHERE;

                    // Sphere scale is diameter in meters
                    marker.scale.x = object[itmRadius].asDouble() * 2.0 / 1000.0;
                    marker.scale.y = marker.scale.x;
                    marker.scale.z = marker.scale.x;
                }
                else if (type.asString() == valCylinder)
                {
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;

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
                    marker.type = visualization_msgs::msg::Marker::CUBE;

                    marker.scale.x = object[itmWidth].asDouble() / 1000.0;
                    marker.scale.y = object[itmWidth].asDouble() / 1000.0;
                    marker.scale.z = object[itmWidth].asDouble() / 1000.0;
                }
                else if (type.asString() == valCuboid)
                {
                    marker.type = visualization_msgs::msg::Marker::CUBE;

                    marker.scale.x = object[itmWidth].asDouble() / 1000.0;
                    marker.scale.z = object[itmHeight].asDouble() / 1000.0;
                    marker.scale.y = object[itmDepth].asDouble() / 1000.0;
                }
                else if (type.asString() == valSingleCustom || type.asString() == valSingle) {
                    // Ensenso and halcon calibration type markers
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    auto grid_size_x = object[itmGridSize][0].asDouble();
                    auto grid_size_y = object[itmGridSize][1].asDouble();
                    auto grid_spacing = object[itmGridSpacing].asDouble();

                    auto plate_width = std::round(0.5 * grid_size_x * grid_spacing / 0.9);
                    auto plate_height = std::round(0.5 * grid_size_y * grid_spacing / 0.9);

                    marker.scale.x = plate_width / 1000.0;
                    marker.scale.y = plate_height / 1000.0;
                    marker.scale.z = 3 / 1000.0;
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
            while (ensenso::ros::ok() && !stop)
            {
                publisher->publish(markers);
                rate.sleep();
            }
        }

        ensenso::ros::NodeHandle nh;
        ensenso::ros::Publisher<visualization_msgs::msg::MarkerArray> publisher;
        ensenso::ros::Rate rate;
        visualization_msgs::msg::MarkerArray markers;
        std::atomic_bool& stop;
    };
}  // namespace

VirtualObjectHandler::VirtualObjectHandler(ensenso::ros::NodeHandle& nh, const std::string& filename,
                                           const std::string& objectsFrame, const std::string& linkFrame,
                                           const std::string& markerTopic, double markerPublishRate)
        : objectsFrame(objectsFrame), linkFrame(linkFrame), tfBuffer(TF2_BUFFER_CTOR_ARGS(nh))
{
    // Read the file contents and assign it to the objects tag
    auto objects = NxLibItem{ itmObjects };
    objects.setJson(readFile(filename));

    // Get the original transforms from file
    for (int i = 0; i < objects.count(); ++i)
    {
        originalTransforms.push_back(transformFromNxLib(objects[i][itmLink]));
        objectFrames.push_back(objects[i][itmLink][itmTarget].asString());
    }

    // Create publisher thread
    if (!markerTopic.empty())
    {
        markerThread = std::thread([=]() {
            VirtualObjectMarkerPublisher publisher{ nh,      markerTopic,  markerPublishRate,
                                                    objects, objectsFrame, stopMarkerThread };
            publisher.run();
        });
    }
}

VirtualObjectHandler::~VirtualObjectHandler()
{
    stopMarkerThread = true;

    if (markerThread.joinable())
    {
        markerThread.join();
    }
}

void VirtualObjectHandler::updateObjectLinks()
{
    // Exit early if we have no work to do
    if (originalTransforms.empty())
    {
        return;
    }

    tf2::Transform cameraTransform;

    // Apply the transform to all of the original transforms
    for (size_t i = 0; i < originalTransforms.size(); ++i)
    {
        // Find transform from the frame in which the objects were defined to the current optical frame
        try
        {
            cameraTransform = fromMsg(tfBuffer.lookupTransform(linkFrame, objectFrames[i], ensenso::ros::Time(0)).transform);
        }
        catch (const tf2::TransformException& e)
        {
            ENSENSO_WARN("Could not look up the virtual object pose due to the TF error: %s", e.what());
            return;
        }

        // Transform object back to original frame
        tf2::Transform objectTransform = cameraTransform * originalTransforms[i];
        NxLibItem objectLink = NxLibItem{ itmObjects }[static_cast<int>(i)][itmLink];
        writeTransformToNxLib(objectTransform.inverse(), objectLink);
    }
}