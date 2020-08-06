#include <ensenso_camera/virtual_object_handler.h>

#include <ensenso_camera/pose_utilities.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>

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
}

VirtualObjectHandler::VirtualObjectHandler(const std::string &filename, const std::string &objectsFrame, const std::string &cameraFrame) :
    objectsFrame(objectsFrame),
    cameraFrame(cameraFrame)
{
    nxLibInitialize(false);

    // Read the file contents and assign it to the objects tag
    auto objects = NxLibItem{ itmObjects };
    objects.setJson(readFile(filename));

    // Get the original poses from file
    for (int i = 0; i < objects.count(); ++i) {
        originalPoses.push_back(poseFromNxLib(objects[i][itmLink]));
    }
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
        cameraPose = fromMsg(tfBuffer.lookupTransform(cameraFrame, objectsFrame, ros::Time(0)).transform);
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
