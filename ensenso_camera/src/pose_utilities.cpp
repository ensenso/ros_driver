#include "ensenso_camera/pose_utilities.h"

#include <string>

bool poseIsValid(const tf::Pose& pose)
{
  auto origin = pose.getOrigin();
  if (std::isnan(origin.x()) || std::isnan(origin.y()) || std::isnan(origin.z()))
  {
    return false;
  }

  auto rotation = pose.getRotation();
  if (std::isnan(rotation.getAngle()))
  {
    return false;
  }

  auto rotationAxis = rotation.getAxis();
  if (std::isnan(rotationAxis.x()) || std::isnan(rotationAxis.y()) || std::isnan(rotationAxis.z()))
  {
    return false;
  }

  return true;
}

void writePoseToNxLib(tf::Pose const& pose, NxLibItem const& node)
{
  // Initialize the node to be empty. This is necessary, because there is a bug in some versions of the NxLib that
  // overwrites the whole transformation node with an identity transformation as soon as a new node in /Links gets
  // created.
  node.setNull();

  if (poseIsValid(pose))
  {
    auto origin = pose.getOrigin();
    node[itmTranslation][0] = origin.x() * 1000;  // ROS transformation is in
                                                  // meters, NxLib expects it to
                                                  // be in millimeters.
    node[itmTranslation][1] = origin.y() * 1000;
    node[itmTranslation][2] = origin.z() * 1000;

    auto rotation = pose.getRotation();
    node[itmRotation][itmAngle] = rotation.getAngle();

    auto rotationAxis = rotation.getAxis();
    node[itmRotation][itmAxis][0] = rotationAxis.x();
    node[itmRotation][itmAxis][1] = rotationAxis.y();
    node[itmRotation][itmAxis][2] = rotationAxis.z();
  }
  else
  {
    // Use an identity transformation as a reasonable default value.
    node[itmTranslation][0] = 0;
    node[itmTranslation][1] = 0;
    node[itmTranslation][2] = 0;

    node[itmRotation][itmAngle] = 0;
    node[itmRotation][itmAxis][0] = 1;
    node[itmRotation][itmAxis][1] = 0;
    node[itmRotation][itmAxis][2] = 0;
  }
}

tf::Pose poseFromNxLib(NxLibItem const& node)
{
  tf::Pose pose;

  tf::Point origin;
  origin.setX(node[itmTranslation][0].asDouble() / 1000);  // NxLib
                                                           // transformation is
                                                           // in millimeters, ROS
                                                           // expects it to be in
                                                           // meters.
  origin.setY(node[itmTranslation][1].asDouble() / 1000);
  origin.setZ(node[itmTranslation][2].asDouble() / 1000);
  pose.setOrigin(origin);

  tf::Vector3 rotationAxis(node[itmRotation][itmAxis][0].asDouble(), node[itmRotation][itmAxis][1].asDouble(),
                           node[itmRotation][itmAxis][2].asDouble());
  tf::Quaternion rotation(rotationAxis, node[itmRotation][itmAngle].asDouble());
  pose.setRotation(rotation);

  return pose;
}

tf::Stamped<tf::Pose> poseFromNxLib(NxLibItem const& node, ros::Time const& timestamp, std::string const& frame)
{
  return tf::Stamped<tf::Pose>(poseFromNxLib(node), timestamp, frame);
}

tf::StampedTransform transformFromPose(geometry_msgs::PoseStamped const& pose, std::string const& childFrame)
{
  tf::StampedTransform transform;

  poseMsgToTF(pose.pose, transform);
  transform.stamp_ = pose.header.stamp;
  transform.frame_id_ = pose.header.frame_id;
  transform.child_frame_id_ = childFrame;

  return transform;
}
