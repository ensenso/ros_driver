#pragma once

#include "ensenso_camera/camera.h"

#include <ensenso_camera/virtual_object_handler.h>

#include <ensenso_camera_msgs/RequestDataAction.h>
#include <ensenso_camera_msgs/LocatePatternAction.h>
#include <ensenso_camera_msgs/ProjectPatternAction.h>
#include <ensenso_camera_msgs/TexturedPointCloudAction.h>
#include <ensenso_camera_msgs/TelecentricProjectionAction.h>

using CalibrateHandEyeServer = QueuedActionServer<ensenso_camera_msgs::CalibrateHandEyeAction>;
using CalibrateWorkspaceServer = QueuedActionServer<ensenso_camera_msgs::CalibrateWorkspaceAction>;
using FitPrimitiveServer = QueuedActionServer<ensenso_camera_msgs::FitPrimitiveAction>;
using RequestDataServer = QueuedActionServer<ensenso_camera_msgs::RequestDataAction>;
using LocatePatternServer = QueuedActionServer<ensenso_camera_msgs::LocatePatternAction>;
using ProjectPatternServer = QueuedActionServer<ensenso_camera_msgs::ProjectPatternAction>;
using TexturedPointCloudServer = QueuedActionServer<ensenso_camera_msgs::TexturedPointCloudAction>;
using TelecentricProjectionServer = QueuedActionServer<ensenso_camera_msgs::TelecentricProjectionAction>;

/**
 * Indicates whether the projector and front light should be turned on or off
 * automatically.
 */
enum ProjectorState
{
  projectorDontCare,  //@< Inherit the projector settings from the parameter set.
  projectorOn,        //@< Enable the projector and disable the front light.
  projectorOff        //@< Enable the front light and disable the projector.
};

class StereoCamera : public Camera
{
private:
  std::string robotFrame;
  std::string wristFrame;

  sensor_msgs::CameraInfoPtr leftCameraInfo;
  sensor_msgs::CameraInfoPtr rightCameraInfo;
  sensor_msgs::CameraInfoPtr leftRectifiedCameraInfo;
  sensor_msgs::CameraInfoPtr rightRectifiedCameraInfo;

  std::unique_ptr<RequestDataServer> requestDataServer;
  std::unique_ptr<CalibrateHandEyeServer> calibrateHandEyeServer;
  std::unique_ptr<CalibrateWorkspaceServer> calibrateWorkspaceServer;
  std::unique_ptr<FitPrimitiveServer> fitPrimitiveServer;
  std::unique_ptr<LocatePatternServer> locatePatternServer;
  std::unique_ptr<ProjectPatternServer> projectPatternServer;
  std::unique_ptr<TexturedPointCloudServer> texturedPointCloudServer;
  std::unique_ptr<TelecentricProjectionServer> telecentricProjectionServer;

  image_transport::CameraPublisher leftRawImagePublisher;
  image_transport::CameraPublisher rightRawImagePublisher;
  image_transport::CameraPublisher leftRectifiedImagePublisher;
  image_transport::CameraPublisher rightRectifiedImagePublisher;
  image_transport::Publisher disparityMapPublisher;
  image_transport::CameraPublisher depthImagePublisher;
  image_transport::Publisher projectedImagePublisher;

  ros::Publisher pointCloudPublisher, pointCloudPublisherColor, projectedPointCloudPublisher;

  // Information that we remember between the different steps of the hand eye
  // calibration.
  // We save the pattern buffer outside of the NxLib, because otherwise we
  // could not use the LocatePattern action while a hand eye calibration is
  // in progress.
  std::string handEyeCalibrationPatternBuffer;
  std::vector<tf2::Transform> handEyeCalibrationRobotPoses;

  // Timeout, in milliseconds, used for capture commands. If <= 0, default timeout is used.
  int captureTimeout;

  // Handler for virtual objects.
  // If set, will update the 'Objects' item based on the current pose of the camera, before each capture.
  std::unique_ptr<ensenso_camera::VirtualObjectHandler> virtualObjectHandler;

public:
  StereoCamera(ros::NodeHandle nh, std::string serial, std::string fileCameraPath, bool fixed, std::string cameraFrame,
               std::string targetFrame, std::string robotFrame, std::string wristFrame, std::string linkFrame,
               int captureTimeout, std::unique_ptr<ensenso_camera::VirtualObjectHandler> virtualObjectHandler=nullptr);

  bool open() override;

  /**
   * Start the action servers. The camera must already be open, otherwise
   * the actions might access parts of the NxLib that are not initialized yet.
   */
  void startServers() const override;

  /**
   * Callback for the `set_parameter` action.
   */
  void onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal) override;
  /**
   * Callback for the `request_data` action.
   */
  void onRequestData(ensenso_camera_msgs::RequestDataGoalConstPtr const& goal);
  /**
   * Callback for the `locate_pattern` action.
   */
  void onLocatePattern(ensenso_camera_msgs::LocatePatternGoalConstPtr const& goal);
  /**
   * Callback for the `project_pattern` action.
   */
  void onProjectPattern(ensenso_camera_msgs::ProjectPatternGoalConstPtr const& goal);
  /**
   * Callback for the `calibrate_hand_eye` action.
   */
  void onCalibrateHandEye(ensenso_camera_msgs::CalibrateHandEyeGoalConstPtr const& goal);
  /**
   * Callback for the `calibrate_workspace` action.
   */
  void onCalibrateWorkspace(ensenso_camera_msgs::CalibrateWorkspaceGoalConstPtr const& goal);
  /**
   * Callback for the `fit_primitive` action.
   */
  void onFitPrimitive(ensenso_camera_msgs::FitPrimitiveGoalConstPtr const& goal);

  /**
   * Callback for the `texture_point_cloud` action.
   */
  void onTexturedPointCloud(ensenso_camera_msgs::TexturedPointCloudGoalConstPtr const& goal);

  /**
   * Callback for the `project_telecentric` action.
   */
  void onTelecentricProjection(ensenso_camera_msgs::TelecentricProjectionGoalConstPtr const& goal);
private:
  /**
   * Save the current settings to the parameter set with the given name.
   *
   * If the projector or front light have been enabled or disabled manually,
   * the flag should be set. It then disabled the automatic control of the
   * projector and front light for this parameter set.
   */
  void saveParameterSet(std::string name, bool projectorWasSet);

  /**
   * Load the parameter set with the given name. If it does not exist yet,
   * it will be created by copying the current default parameters.
   *
   * The projector and front light will be enabled according to the given
   * flag, unless they have been manually enabled or disabled for the
   * current parameter set.
   */
  void loadParameterSet(std::string name, ProjectorState projector = projectorDontCare);

  /**
   * Capture a new pair of images. Returns the timestamp of the (first) captured image.
   */
  ros::Time capture() const override;

  /**
   * Try to collect patterns on the current images. For the command to be
   * successful, the patterns must be decodable and visible in both cameras.
   */
  std::vector<StereoCalibrationPattern> collectPattern(bool clearBuffer = false) const;

  /**
   * Read the camera calibration from the NxLib and write it into a CameraInfo
   * message.
   *
   * @param info  The CameraInfo message to which the calibration should be
   *              written.
   * @param right Whether to use the calibration from the right camera instead
   *              of the left one.
   */
  void fillCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info, bool right, bool rectified = false) const;
  /**
   * Update the cached CameraInfo messages that will be published together
   * with the images.
   */
  void updateCameraInfo() override;

  ensenso_camera_msgs::ParameterPtr readParameter(std::string const& key) const override;
  void writeParameter(ensenso_camera_msgs::Parameter const& parameter) override;
};
