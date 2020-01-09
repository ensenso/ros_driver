#include <vector>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

namespace ensenso_camera {
    class VirtualObjectHandler {
    public:
        VirtualObjectHandler(const std::string &filename, const std::string &objectsFrame, const std::string &cameraFrame);

        void updateObjectLinks();

    private:
        /// Original object poses in the base frame
        std::vector<tf2::Transform> originalPoses;

        std::string objectsFrame;    ///< Frame in which objects are defined
        std::string cameraFrame;     ///< Optical frame of the camera

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener{ tfBuffer };
    };
}
