#pragma once

#include "ensenso_camera/camera.h"

#include <ensenso_camera_msgs/RequestDataMonoAction.h>
#include <ensenso_camera_msgs/LocatePatternMonoAction.h>

using RequestDataMonoServer = QueuedActionServer<ensenso_camera_msgs::RequestDataMonoAction>;
using LocatePatternMonoServer = QueuedActionServer<ensenso_camera_msgs::LocatePatternMonoAction>;

class MonoCamera : public Camera
{
private:
  sensor_msgs::CameraInfoPtr cameraInfo;
  sensor_msgs::CameraInfoPtr rectifiedCameraInfo;

  std::unique_ptr<RequestDataMonoServer> requestDataServer;
  std::unique_ptr<LocatePatternMonoServer> locatePatternServer;

  image_transport::CameraPublisher rawImagePublisher;
  image_transport::CameraPublisher rectifiedImagePublisher;

public:
  MonoCamera(ros::NodeHandle nh, CameraParameters params);

  void init() override;

  ros::Time capture() const override;

  void onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal) override;

  /**
   * Callback for the `request_data` action.
   */
  void onRequestData(ensenso_camera_msgs::RequestDataMonoGoalConstPtr const& goal);

  /**
   * Callback for the `locate_pattern` action.
   */
  void onLocatePattern(ensenso_camera_msgs::LocatePatternMonoGoalConstPtr const& goal);

private:
  void startServers() const override;

  void updateCameraInfo() override;

  geometry_msgs::TransformStamped estimatePatternPose(ros::Time imageTimestamp = ros::Time::now(),
                                                      std::string const& targetFrame = "",
                                                      bool latestPatternOnly = false) const override;

  std::vector<geometry_msgs::TransformStamped> estimatePatternPoses(ros::Time imageTimestamp = ros::Time::now(),
                                                                    std::string const& targetFrame = "") const override;

  /**
   * Advertise all camera topics.
   */
  void advertiseTopics();

  /**
   * Read the camera calibration from the NxLib and write it into a CameraInfo message.
   */
  void fillCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info, bool rectified = false);

  /**
   * Runs the NxLib's collectPattern command and returns a vector of MonoCalibrationPatterns. The result is empty if no
   * pattern was found or if the pattern is / patterns are not decodable. Otherwise the result contains the found
   * patterns.
   */
  std::vector<MonoCalibrationPattern> collectPattern(bool clearBuffer = false) const;
};
