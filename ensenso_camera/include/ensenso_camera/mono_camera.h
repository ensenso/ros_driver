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
  MonoCamera(ros::NodeHandle nh, std::string serial, std::string fileCameraPath, bool fixed, std::string cameraFrame,
             std::string targetFrame, std::string linkFrame);

  bool open() override;

  void startServers() const override;

  /**
   * Callback for the `request_data` action.
   */
  void onRequestData(ensenso_camera_msgs::RequestDataMonoGoalConstPtr const& goal);

  void onLocatePattern(ensenso_camera_msgs::LocatePatternMonoGoalConstPtr const& goal);

  void onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal) override;

  ros::Time capture() const override;

private:
  void updateCameraInfo() override;
  void fillCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info);

  std::vector<MonoCalibrationPattern> collectPattern(bool clearBuffer = false) const;

  /**
   * Estimate the pose of a pattern in the given TF frame. The pattern must
   * already be contained in the pattern buffer (that is, you should call
   * collectPattern before this function).
   *
   * When the latestPatternOnly flag is set, the estimated position will be
   * the one of the latest pattern in the buffer. Otherwise the function
   * assumes that all observations are of the same pattern. It will then
   * average their positions to increase the accuracy of the pose estimation.
   */
  geometry_msgs::TransformStamped estimatePatternPose(ros::Time imageTimestamp = ros::Time::now(),
                                                      std::string const& targetFrame = "",
                                                      bool latestPatternOnly = false) const override;

  /**
   * Estimate the pose of each pattern in the pattern buffer for mono cameras.
   */
  std::vector<geometry_msgs::TransformStamped> estimatePatternPoses(ros::Time imageTimestamp = ros::Time::now(),
                                                                    std::string const& targetFrame = "") const override;
};
