#include "ensenso_camera/pose_utilities.h"

#include <string>

namespace
{
bool isValid(tf2::Vector3 const& vector)
{
  return (!std::isnan(vector.x()) && !std::isnan(vector.y()) && !std::isnan(vector.z()));
}

tf2Scalar fixZeroValue(double value, double epsilon = 0.00001)
{
  if (std::abs(value) <= epsilon)
  {
    return tf2Scalar(0.0);
  }
  return tf2Scalar(value);
}

tf2::Vector3 vector3FromNxLib(NxLibItem const& node)
{
  double x = node[0].asDouble();
  double y = node[1].asDouble();
  double z = node[2].asDouble();
  return tf2::Vector3(tf2Scalar(x), tf2Scalar(y), tf2Scalar(z));
}
}  // namespace

bool isValid(tf2::Transform const& pose)
{
  auto origin = pose.getOrigin();
  if (!isValid(origin))
  {
    return false;
  }

  auto rotation = pose.getRotation();
  if (std::isnan(rotation.getAngle()))
  {
    return false;
  }

  auto rotationAxis = rotation.getAxis();
  return isValid(rotationAxis);
}

bool isValid(geometry_msgs::Transform const& pose)
{
  return isValid(fromMsg(pose));
}

bool isIdentity(tf2::Transform const& pose)
{
  tf2::Matrix3x3 basis(fixZeroValue(pose.getBasis().getRow(0).getX()), fixZeroValue(pose.getBasis().getRow(0).getY()),
                       fixZeroValue(pose.getBasis().getRow(0).getZ()), fixZeroValue(pose.getBasis().getRow(1).getX()),
                       fixZeroValue(pose.getBasis().getRow(1).getY()), fixZeroValue(pose.getBasis().getRow(1).getZ()),
                       fixZeroValue(pose.getBasis().getRow(2).getX()), fixZeroValue(pose.getBasis().getRow(2).getY()),
                       fixZeroValue(pose.getBasis().getRow(2).getZ()));

  tf2::Vector3 origin(fixZeroValue(pose.getOrigin().getX()), fixZeroValue(pose.getOrigin().getY()),
                      fixZeroValue(pose.getOrigin().getZ()));

  return (basis == tf2::Matrix3x3::getIdentity() && origin.isZero());
}

void writePoseToNxLib(tf2::Transform const& pose, NxLibItem const& node)
{
  // Initialize the node to be empty. This is necessary, because there is a bug in some versions of the NxLib that
  // overwrites the whole transformation node with an identity transformation as soon as a new node in /Links gets
  // created.
  if (node.path.find("ViewPose") == std::string::npos)
  {
    // The ensenso SDK 2.2.x has a structure locked ViewPose item in the global params. So it cannot be set to null.
    node.setNull();
  }

  if (isValid(pose))
  {
    // ROS transformation is in meters, NxLib expects it to be in millimeters.
    auto origin = pose.getOrigin();
    origin *= 1000;
    node[itmTranslation][0] = origin.x();
    node[itmTranslation][1] = origin.y();
    node[itmTranslation][2] = origin.z();

    auto rotation = pose.getRotation();
    node[itmRotation][itmAngle] = rotation.getAngle();

    auto rotationAxis = rotation.getAxis();
    node[itmRotation][itmAxis][0] = rotationAxis.x();
    node[itmRotation][itmAxis][1] = rotationAxis.y();
    node[itmRotation][itmAxis][2] = rotationAxis.z();
  }
  else
  {
    ROS_ERROR("Given pose is not valid for writing to the NxLib, using identity transform instead");
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

namespace tf2
{
void convertMsg(geometry_msgs::Transform const& transform, geometry_msgs::Pose& pose)
{
  pose.orientation = transform.rotation;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
}

void convertMsg(geometry_msgs::Pose const& pose, geometry_msgs::Transform& transform)
{
  transform.rotation = pose.orientation;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
}

void convertMsg(geometry_msgs::TransformStamped const& transform, geometry_msgs::PoseStamped& pose)
{
  convertMsg(transform.transform, pose.pose);
  pose.header = transform.header;
}

void convertMsg(geometry_msgs::PoseStamped const& pose, geometry_msgs::TransformStamped& transform)
{
  convertMsg(pose.pose, transform.transform);
  transform.header = pose.header;
}
}  // namespace tf2

tf2::Transform poseFromNxLib(NxLibItem const& node)
{
  tf2::Transform pose;

  // NxLib transformation is in millimeters, ROS expects it to be in meters.
  tf2::Vector3 origin = vector3FromNxLib(node[itmTranslation]);
  origin /= 1000;
  pose.setOrigin(origin);

  tf2::Quaternion rotation(vector3FromNxLib(node[itmRotation][itmAxis]), node[itmRotation][itmAngle].asDouble());
  pose.setRotation(rotation);

  return pose;
}

geometry_msgs::TransformStamped poseFromNxLib(NxLibItem const& node, std::string const& parentFrame,
                                              std::string const& childFrame, ros::Time timestamp)
{
  geometry_msgs::TransformStamped stampedTransform;
  stampedTransform.header.stamp = timestamp;
  stampedTransform.header.frame_id = parentFrame;
  stampedTransform.child_frame_id = childFrame;

  tf2::Transform transform = poseFromNxLib(node);
  tf2::convert(transform, stampedTransform.transform);
  return stampedTransform;
}

geometry_msgs::TransformStamped transformFromPose(geometry_msgs::PoseStamped const& pose, std::string const& childFrame)
{
  geometry_msgs::TransformStamped transform;
  // TODO Check if it is a problem if header.id is also copied in convertMsg()
  tf2::convertMsg(pose, transform);
  transform.child_frame_id = childFrame;

  return transform;
}

geometry_msgs::PoseStamped stampedPoseFromTransform(geometry_msgs::TransformStamped const& transform)
{
  geometry_msgs::PoseStamped pose;
  // TODO Check if it is a problem if header.id is also copied in convertMsg()
  tf2::convertMsg(transform, pose);

  return pose;
}

tf2::Transform fromStampedMessage(geometry_msgs::TransformStamped const& tStamped)
{
  tf2::Transform transform;
  tf2::Quaternion quat;
  tf2::convert(tStamped.transform.rotation, quat);
  transform.setRotation(quat);
  tf2::Vector3 trans;
  tf2::convert(tStamped.transform.translation, trans);
  transform.setOrigin(trans);

  return transform;
}

tf2::Transform fromStampedMessage(geometry_msgs::PoseStamped const& pStamped)
{
  geometry_msgs::TransformStamped tStamped;
  tf2::convertMsg(pStamped, tStamped);

  tf2::Transform transform;
  tf2::convert(tStamped.transform, transform);

  return transform;
}

tf2::Transform fromMsg(geometry_msgs::Transform const& t)
{
  tf2::Transform transform;
  tf2::convert(t, transform);

  return transform;
}

tf2::Transform fromMsg(geometry_msgs::Pose const& p)
{
  geometry_msgs::Transform t;
  tf2::convertMsg(p, t);

  tf2::Transform transform;
  tf2::convert(t, transform);

  return transform;
}

geometry_msgs::Pose poseFromTransform(tf2::Transform const& transform)
{
  geometry_msgs::Transform t;
  tf2::convert(transform, t);

  geometry_msgs::Pose pose;
  tf2::convertMsg(t, pose);

  return pose;
}

geometry_msgs::TransformStamped fromTfTransform(tf2::Transform const& transform, std::string parentFrame,
                                                std::string childFrame, ros::Time timestamp)
{
  geometry_msgs::TransformStamped tStamped;
  tf2::convert(transform, tStamped.transform);
  tStamped.header.stamp = timestamp;
  tStamped.header.frame_id = std::move(parentFrame);
  tStamped.child_frame_id = std::move(childFrame);

  return tStamped;
}

tf2::Transform getLatestTransform(tf2_ros::Buffer const& tfBuffer, std::string const& cameraFrame,
                                  std::string const& targetFrame)
{
  tf2::Transform transform;
  try
  {
    geometry_msgs::TransformStamped stTransform;
    stTransform = tfBuffer.lookupTransform(cameraFrame, targetFrame, ros::Time(0));

    transform = fromMsg(stTransform.transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  return transform;
}
