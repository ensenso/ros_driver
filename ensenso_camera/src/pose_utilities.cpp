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

bool isValid(tf2::Transform const& transform)
{
  auto origin = transform.getOrigin();
  if (!isValid(origin))
  {
    return false;
  }

  auto rotation = transform.getRotation();
  if (std::isnan(rotation.getAngle()))
  {
    return false;
  }

  auto rotationAxis = rotation.getAxis();
  return isValid(rotationAxis);
}

bool isValid(TransformMsg const& transform)
{
  return isValid(fromMsg(transform));
}

bool isIdentity(tf2::Transform const& transform)
{
  tf2::Matrix3x3 basis(
      fixZeroValue(transform.getBasis().getRow(0).getX()), fixZeroValue(transform.getBasis().getRow(0).getY()),
      fixZeroValue(transform.getBasis().getRow(0).getZ()), fixZeroValue(transform.getBasis().getRow(1).getX()),
      fixZeroValue(transform.getBasis().getRow(1).getY()), fixZeroValue(transform.getBasis().getRow(1).getZ()),
      fixZeroValue(transform.getBasis().getRow(2).getX()), fixZeroValue(transform.getBasis().getRow(2).getY()),
      fixZeroValue(transform.getBasis().getRow(2).getZ()));

  tf2::Vector3 origin(fixZeroValue(transform.getOrigin().getX()), fixZeroValue(transform.getOrigin().getY()),
                      fixZeroValue(transform.getOrigin().getZ()));

  return (basis == tf2::Matrix3x3::getIdentity() && origin.isZero());
}

void writeTransformToNxLib(tf2::Transform const& transform, NxLibItem const& node)
{
  // Initialize the node to be empty. This is necessary, because there is a bug in some versions of the NxLib that
  // overwrites the whole transformation node with an identity transformation as soon as a new node in /Links gets
  // created.
  if (node.path.find("ViewPose") == std::string::npos)
  {
    // The ensenso SDK 2.2.x has a structure locked ViewPose item in the global params. So it cannot be set to null.
    node.setNull();
  }

  if (isValid(transform))
  {
    // ROS transformation is in meters, NxLib expects it to be in millimeters.
    auto origin = transform.getOrigin();
    origin *= 1000;
    node[itmTranslation][0] = origin.x();
    node[itmTranslation][1] = origin.y();
    node[itmTranslation][2] = origin.z();

    auto rotation = transform.getRotation();
    node[itmRotation][itmAngle] = rotation.getAngle();

    auto rotationAxis = rotation.getAxis();
    node[itmRotation][itmAxis][0] = rotationAxis.x();
    node[itmRotation][itmAxis][1] = rotationAxis.y();
    node[itmRotation][itmAxis][2] = rotationAxis.z();
  }
  else
  {
    ROS_ERROR("Given transform is not valid for writing to the NxLib, using identity transform instead");
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
void convertMsg(TransformMsg const& transform, PoseMsg& pose)
{
  pose.orientation = transform.rotation;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
}

void convertMsg(PoseMsg const& pose, TransformMsg& transform)
{
  transform.rotation = pose.orientation;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
}

void convertMsg(StampedTransformMsg const& transform, StampedPoseMsg& pose)
{
  convertMsg(transform.transform, pose.pose);
  pose.header = transform.header;
}

void convertMsg(StampedPoseMsg const& pose, StampedTransformMsg& transform)
{
  convertMsg(pose.pose, transform.transform);
  transform.header = pose.header;
}
}  // namespace tf2

tf2::Transform transformFromNxLib(NxLibItem const& node)
{
  tf2::Transform transform;

  // NxLib transformation is in millimeters, ROS expects it to be in meters.
  tf2::Vector3 origin = vector3FromNxLib(node[itmTranslation]);
  origin /= 1000;
  transform.setOrigin(origin);

  tf2::Quaternion rotation(vector3FromNxLib(node[itmRotation][itmAxis]), node[itmRotation][itmAngle].asDouble());
  transform.setRotation(rotation);

  return transform;
}

StampedPoseMsg stampedPoseFromNxLib(NxLibItem const& node, std::string const& parentFrame, ros::Time timestamp)
{
  StampedTransformMsg stampedTransform;
  tf2::Transform transform = transformFromNxLib(node);
  tf2::convert(transform, stampedTransform.transform);

  StampedPoseMsg stampedPose;
  tf2::convertMsg(stampedTransform, stampedPose);
  stampedPose.header.frame_id = parentFrame;
  stampedPose.header.stamp = timestamp;

  return stampedPose;
}

StampedTransformMsg transformFromPose(StampedPoseMsg const& pose, std::string const& childFrame)
{
  StampedTransformMsg transform;
  // TODO Check if it is a problem if header.id is also copied in convertMsg()
  tf2::convertMsg(pose, transform);
  transform.child_frame_id = childFrame;

  return transform;
}

StampedPoseMsg poseFromTransform(StampedTransformMsg const& transform)
{
  StampedPoseMsg pose;
  // TODO Check if it is a problem if header.id is also copied in convertMsg()
  tf2::convertMsg(transform, pose);

  return pose;
}

tf2::Transform fromMsg(StampedTransformMsg const& tStamped)
{
  tf2::Transform transform;
  tf2::convert(tStamped.transform, transform);

  return transform;
}

tf2::Transform fromMsg(StampedPoseMsg const& pStamped)
{
  StampedTransformMsg tStamped;
  tf2::convertMsg(pStamped, tStamped);

  tf2::Transform transform;
  tf2::convert(tStamped.transform, transform);

  return transform;
}

tf2::Transform fromMsg(TransformMsg const& t)
{
  tf2::Transform transform;
  tf2::convert(t, transform);

  return transform;
}

tf2::Transform fromMsg(PoseMsg const& p)
{
  TransformMsg t;
  tf2::convertMsg(p, t);

  tf2::Transform transform;
  tf2::convert(t, transform);

  return transform;
}

PoseMsg poseFromTransform(tf2::Transform const& transform)
{
  TransformMsg t;
  tf2::convert(transform, t);

  PoseMsg pose;
  tf2::convertMsg(t, pose);

  return pose;
}

StampedTransformMsg fromTf(tf2::Transform const& transform, std::string parentFrame, std::string childFrame,
                           ros::Time timestamp)
{
  StampedTransformMsg tStamped;
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
    StampedTransformMsg tStamped;
    tStamped = tfBuffer.lookupTransform(cameraFrame, targetFrame, ros::Time(0));

    transform = fromMsg(tStamped.transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  return transform;
}
