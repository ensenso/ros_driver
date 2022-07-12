#pragma once

#include "ensenso_camera/camera.h"

#ifdef ROS2
#include <ensenso_camera_msgs/action/locate_pattern_mono.hpp>
#include <ensenso_camera_msgs/action/request_data_mono.hpp>
#else
#include <ensenso_camera_msgs/LocatePatternMonoAction.h>
#include <ensenso_camera_msgs/RequestDataMonoAction.h>
#endif

USING_ENSENSO_CAMERA_ACTION_WITH_SERVER(LocatePatternMono)
USING_ENSENSO_CAMERA_ACTION_WITH_SERVER(RequestDataMono)

USING_ENSENSO_CAMERA_MSG(ImagePoint)

class MonoCamera : public Camera
{
private:
  sensor_msgs::msg::CameraInfoPtr cameraInfo;
  sensor_msgs::msg::CameraInfoPtr rectifiedCameraInfo;

  std::unique_ptr<RequestDataMonoServer> requestDataServer;
  std::unique_ptr<LocatePatternMonoServer> locatePatternServer;

  image_transport::CameraPublisher rawImagePublisher;
  image_transport::CameraPublisher rectifiedImagePublisher;

public:
  MonoCamera(ensenso::ros::NodeHandle& nh, CameraParameters params);

  void init() override;

  ensenso::ros::Time capture() const override;

  void onSetParameter(ensenso::action::SetParameterGoalConstPtr const& goal) override;

  /**
   * Callback for the `request_data` action.
   */
  void onRequestData(ensenso::action::RequestDataMonoGoalConstPtr const& goal);

  /**
   * Callback for the `locate_pattern` action.
   */
  void onLocatePattern(ensenso::action::LocatePatternMonoGoalConstPtr const& goal);

private:
  void startServers() const override;

  void updateCameraInfo() override;

  geometry_msgs::msg::PoseStamped estimatePatternPose(ensenso::ros::Time imageTimestamp,
                                                      std::string const& targetFrame = "",
                                                      bool latestPatternOnly = false) const override;

  std::vector<geometry_msgs::msg::PoseStamped> estimatePatternPoses(ensenso::ros::Time imageTimestamp,
                                                                    std::string const& targetFrame = "") const override;

  /**
   * Advertise all camera topics.
   */
  void advertiseTopics();

  /**
   * Read the camera calibration from the NxLib and write it into a CameraInfo message.
   */
  void fillCameraInfoFromNxLib(sensor_msgs::msg::CameraInfoPtr const& info, bool rectified = false);

  /**
   * Runs the NxLib's collectPattern command and returns a vector of MonoCalibrationPatterns. The result is empty if no
   * pattern was found or if the pattern is / patterns are not decodable. Otherwise the result contains the found
   * patterns.
   */
  std::vector<MonoCalibrationPattern> collectPattern(bool clearBuffer = false) const;
};
