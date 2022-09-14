#pragma once

#include "ensenso_camera/camera.h"

#include "ensenso_camera/ros2/pcl.h"

#include "ensenso_camera/ros2/ensenso_msgs/calibrate_hand_eye.h"
#include "ensenso_camera/ros2/ensenso_msgs/calibrate_workspace.h"
#include "ensenso_camera/ros2/ensenso_msgs/fit_primitive.h"
#include "ensenso_camera/ros2/ensenso_msgs/locate_pattern.h"
#include "ensenso_camera/ros2/ensenso_msgs/primitive.h"
#include "ensenso_camera/ros2/ensenso_msgs/project_pattern.h"
#include "ensenso_camera/ros2/ensenso_msgs/request_data.h"
#include "ensenso_camera/ros2/ensenso_msgs/telecentric_projection.h"
#include "ensenso_camera/ros2/ensenso_msgs/textured_point_cloud.h"

/**
 * Indicates whether the projector and front light should be turned on or off automatically.
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
  sensor_msgs::msg::CameraInfoPtr leftCameraInfo;
  sensor_msgs::msg::CameraInfoPtr rightCameraInfo;
  sensor_msgs::msg::CameraInfoPtr leftRectifiedCameraInfo;
  sensor_msgs::msg::CameraInfoPtr rightRectifiedCameraInfo;

  std::unique_ptr<CalibrateHandEyeServer> calibrateHandEyeServer;
  std::unique_ptr<CalibrateWorkspaceServer> calibrateWorkspaceServer;
  std::unique_ptr<FitPrimitiveServer> fitPrimitiveServer;
  std::unique_ptr<LocatePatternServer> locatePatternServer;
  std::unique_ptr<ProjectPatternServer> projectPatternServer;
  std::unique_ptr<RequestDataServer> requestDataServer;
  std::unique_ptr<TexturedPointCloudServer> texturedPointCloudServer;
  std::unique_ptr<TelecentricProjectionServer> telecentricProjectionServer;

  image_transport::CameraPublisher leftRawImagePublisher;
  image_transport::CameraPublisher rightRawImagePublisher;
  image_transport::CameraPublisher leftRectifiedImagePublisher;
  image_transport::CameraPublisher rightRectifiedImagePublisher;
  image_transport::CameraPublisher disparityMapPublisher;
  image_transport::CameraPublisher depthImagePublisher;
  image_transport::Publisher projectedImagePublisher;

  PointCloudPublisher<ensenso::pcl::PointCloud> pointCloudPublisher;
  PointCloudPublisher<ensenso::pcl::PointCloudNormals> pointCloudNormalsPublisher;
  PointCloudPublisher<ensenso::pcl::PointCloudColored> pointCloudColoredPublisher;
  PointCloudPublisher<ensenso::pcl::PointCloud> pointCloudProjectedPublisher;

  // Information that we remember between the different steps of the hand-eye calibration. We save the pattern buffer
  // outside of the NxLib, because otherwise we could not use the LocatePattern action while a hand-eye calibration is
  // in progress.
  std::string handEyeCalibrationPatternBuffer;
  std::vector<tf2::Transform> handEyeCalibrationRobotTransforms;

public:
  StereoCamera(ensenso::ros::NodeHandle& nh, CameraParameters params);

  void init() override;

  void onSetParameter(ensenso::action::SetParameterGoalConstPtr const& goal) override;

  /**
   * Callback for the `request_data` action.
   */
  void onRequestData(ensenso::action::RequestDataGoalConstPtr const& goal);

  /**
   * Callback for the `locate_pattern` action.
   */
  void onLocatePattern(ensenso::action::LocatePatternGoalConstPtr const& goal);

  /**
   * Callback for the `project_pattern` action.
   */
  void onProjectPattern(ensenso::action::ProjectPatternGoalConstPtr const& goal);

  /**
   * Callback for the `calibrate_hand_eye` action.
   */
  void onCalibrateHandEye(ensenso::action::CalibrateHandEyeGoalConstPtr const& goal);

  /**
   * Callback for the `calibrate_workspace` action.
   */
  void onCalibrateWorkspace(ensenso::action::CalibrateWorkspaceGoalConstPtr const& goal);

  /**
   * Callback for the `fit_primitive` action.
   */
  void onFitPrimitive(ensenso::action::FitPrimitiveGoalConstPtr const& goal);

  /**
   * Callback for the `texture_point_cloud` action.
   */
  void onTexturedPointCloud(ensenso::action::TexturedPointCloudGoalConstPtr const& goal);

  /**
   * Callback for the `project_telecentric` action.
   */
  void onTelecentricProjection(ensenso::action::TelecentricProjectionGoalConstPtr const& goal);

private:
  void updateCameraTypeSpecifics() override;

  void startServers() const override;

  void updateCameraInfo() override;

  geometry_msgs::msg::PoseStamped estimatePatternPose(ensenso::ros::Time imageTimestamp,
                                                      std::string const& targetFrame = "",
                                                      bool latestPatternOnly = false) const override;

  std::vector<geometry_msgs::msg::PoseStamped> estimatePatternPoses(ensenso::ros::Time imageTimestamp,
                                                                    std::string const& targetFrame = "") const override;

  ensenso::ros::Time capture() const override;

  ensenso::msg::ParameterPtr readParameter(std::string const& key) const override;

  void writeParameter(ensenso::msg::Parameter const& parameter) override;

  /**
   * Advertise all camera topics.
   */
  void advertiseTopics();

  /**
   * Save the current settings to the parameter set with the given name.
   *
   * If the projector or front light have been enabled or disabled manually, the flag should be set. It then disabled
   * the automatic control of the projector and front light for this parameter set.
   */
  void saveParameterSet(std::string name, bool projectorWasSet);

  /**
   * Load the parameter set with the given name. If it does not exist yet, it will be created by copying the current
   * default parameters.
   *
   * The projector and front light will be enabled according to the given flag, unless they have been manually enabled
   * or disabled for the current parameter set.
   */
  void loadParameterSet(std::string name, ProjectorState projector = projectorDontCare);

  /**
   * Grab the timestamp of the last captured (raw) image. Handle the different image sources across different camera
   * models (file camera, S-Series, XR-Series or normal stereo camera).
   */
  ensenso::ros::Time timestampOfCapturedImage() const;

  /**
   * Try to collect patterns on the current images. For the command to be successful, the patterns must be decodable and
   * visible in both cameras.
   */
  std::vector<StereoCalibrationPattern> collectPattern(bool clearBuffer = false) const;

  /**
   * Read the camera calibration from the NxLib and write it into a CameraInfo message.
   *
   * When the right flag is set, use the calibration from the right camera instead from the left.
   * The rectified flag indicates whether the images are already rectified.
   */
  void fillCameraInfoFromNxLib(sensor_msgs::msg::CameraInfoPtr const& info, bool right, bool rectified = false) const;

  /**
   * Return whether this camera is an S-series camera.
   */
  bool isSSeries() const;

  /**
   * Return whether this camera is an XR-Series camera.
   */
  bool isXrSeries() const;

  /**
   * Return whether this camera has a right camera sensor.
   */
  bool hasRightCamera() const;

  /**
   * Return whether this cameras has/stores raw images.
   */
  bool hasRawImages() const;

  /**
   * Return whether this camera downloads the raw/rectified images.
   */
  bool hasDownloadedImages() const;

  /**
   * Return whether this camera has a disparity map.
   */
  bool hasDisparityMap() const;

  /**
   * Add the NxLib internal disparity map offset to the given camera info.
   */
  void addDisparityMapOffset(sensor_msgs::msg::CameraInfoPtr const& info) const;
};
