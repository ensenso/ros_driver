#pragma once

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ensenso_camera_msgs/AccessTreeAction.h>
#include <ensenso_camera_msgs/CalibrateHandEyeAction.h>
#include <ensenso_camera_msgs/CalibrateWorkspaceAction.h>
#include <ensenso_camera_msgs/ExecuteCommandAction.h>
#include <ensenso_camera_msgs/GetParameterAction.h>
#include <ensenso_camera_msgs/LocatePatternAction.h>
#include <ensenso_camera_msgs/ProjectPatternAction.h>
#include <ensenso_camera_msgs/RequestDataAction.h>
#include <ensenso_camera_msgs/SetParameterAction.h>

#include "ensenso_camera/calibration_pattern.h"
#include "ensenso_camera/point_cloud_utilities.h"
#include "ensenso_camera/queued_action_server.h"

#include "nxLib.h"

using AccessTreeServer = QueuedActionServer<ensenso_camera_msgs::AccessTreeAction>;
using CalibrateHandEyeServer = QueuedActionServer<ensenso_camera_msgs::CalibrateHandEyeAction>;
using CalibrateWorkspaceServer = QueuedActionServer<ensenso_camera_msgs::CalibrateWorkspaceAction>;
using ExecuteCommandServer = QueuedActionServer<ensenso_camera_msgs::ExecuteCommandAction>;
using GetParameterServer = QueuedActionServer<ensenso_camera_msgs::GetParameterAction>;
using LocatePatternServer = QueuedActionServer<ensenso_camera_msgs::LocatePatternAction>;
using ProjectPatternServer = QueuedActionServer<ensenso_camera_msgs::ProjectPatternAction>;
using RequestDataServer = QueuedActionServer<ensenso_camera_msgs::RequestDataAction>;
using SetParameterServer = QueuedActionServer<ensenso_camera_msgs::SetParameterAction>;

/**
 * A set of parameters that can be used by the different actions of the camera
 * node.
 */
struct ParameterSet
{
  /**
   * An NxLib node in which we store the NxLib camera parameters for this
   * parameter set.
   */
  NxLibItem node;

  /**
   * Whether the 3D region of interest is enabled for this parameter set.
   */
  bool useROI = false;

  /**
   * A 3D region of interest by which the point cloud is filtered before it is
   * published.
   */
  PointCloudROI roi;

  /**
   * Whether projector and front light are managed automatically for this
   * parameter set. This gets disabled as soon as either the projector or the
   * front light are set manually.
   */
  bool autoProjector = true;

  ParameterSet(std::string const& name, NxLibItem const& defaultParameters);
};

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

class Camera
{
private:
  std::string serial;
  NxLibItem cameraNode;

  bool isFileCamera;
  std::string fileCameraPath;
  // Whether the camera was created by this node. If that is the case, we will
  // delete it again after it got closed.
  bool createdFileCamera;

  // Whether the camera is fixed in the world or moves with a robot.
  bool fixed;

  std::string cameraFrame;
  std::string targetFrame;
  std::string robotFrame;
  std::string wristFrame;

  // Controls parallel access to the NxLib.
  mutable std::mutex nxLibMutex;

  sensor_msgs::CameraInfoPtr leftCameraInfo;
  sensor_msgs::CameraInfoPtr rightCameraInfo;
  sensor_msgs::CameraInfoPtr leftRectifiedCameraInfo;
  sensor_msgs::CameraInfoPtr rightRectifiedCameraInfo;

  tf::TransformListener transformListener;
  tf::TransformBroadcaster transformBroadcaster;

  std::unique_ptr<AccessTreeServer> accessTreeServer;
  std::unique_ptr<ExecuteCommandServer> executeCommandServer;
  std::unique_ptr<GetParameterServer> getParameterServer;
  std::unique_ptr<SetParameterServer> setParameterServer;
  std::unique_ptr<RequestDataServer> requestDataServer;
  std::unique_ptr<LocatePatternServer> locatePatternServer;
  std::unique_ptr<ProjectPatternServer> projectPatternServer;
  std::unique_ptr<CalibrateHandEyeServer> calibrateHandEyeServer;
  std::unique_ptr<CalibrateWorkspaceServer> calibrateWorkspaceServer;

  image_transport::CameraPublisher leftRawImagePublisher;
  image_transport::CameraPublisher rightRawImagePublisher;
  image_transport::CameraPublisher leftRectifiedImagePublisher;
  image_transport::CameraPublisher rightRectifiedImagePublisher;
  image_transport::Publisher disparityMapPublisher;

  ros::Publisher pointCloudPublisher;

  ros::Publisher statusPublisher;
  ros::Timer statusTimer;

  // Contains a parameter tree that is used for creating new parameter sets.
  NxLibItem defaultParameters;

  std::map<std::string, ParameterSet> parameterSets;
  std::string currentParameterSet;

  mutable std::map<std::string, tf::StampedTransform> transformationCache;

  // Information that we remember between the different steps of the hand eye
  // calibration.
  // We save the pattern buffer outside of the NxLib, because otherwise we
  // could not use the LocatePattern action while a hand eye calibration is
  // in progress.
  std::string handEyeCalibrationPatternBuffer;
  std::vector<tf::Pose> handEyeCalibrationRobotPoses;

public:
  Camera(ros::NodeHandle nh, std::string const& serial, std::string const& fileCameraPath, bool fixed,
         std::string const& cameraFrame, std::string const& targetFrame, std::string const& robotFrame,
         std::string const& wristFrame);

  bool open();
  void close();

  /**
   * Start the action servers. The camera must already be open, otherwise
   * the actions might access parts of the NxLib that are not initialized yet.
   */
  void startServers() const;

  /**
   * Load the camera settings from the given JSON file. The resulting
   * parameters will also be saved as the default values for new parameter
   * sets.
   *
   * Returns true if the settings could be applied successfully.
   */
  bool loadSettings(std::string const& jsonFile, bool saveAsDefaultParameters = false);

  /**
   * Callback for the `access_tree` action.
   */
  void onAccessTree(ensenso_camera_msgs::AccessTreeGoalConstPtr const& goal);
  /**
   * Callback for the `execute_command` action.
   */
  void onExecuteCommand(ensenso_camera_msgs::ExecuteCommandGoalConstPtr const& goal);

  /**
   * Callback for the `get_parameter` action.
   */
  void onGetParameter(ensenso_camera_msgs::GetParameterGoalConstPtr const& goal);
  /**
   * Callback for the `set_parameter` action.
   */
  void onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal);

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

private:
  /**
   * Check whether the camera is available for opening.
   */
  bool cameraIsAvailable() const;

  /**
   * Check whether the camera is still open in the NxLib.
   */
  bool cameraIsOpen() const;
  /**
   * Publish a diagnostic message indicating whether the camera is still open
   * and usable in the NxLib.
   */
  void publishStatus(ros::TimerEvent const& event) const;

  /**
   * Read the current parameters from the camera node and store them as the
   * default parameter set that will later be used for creating new parameter
   * sets.
   */
  void saveDefaultParameterSet();

  /**
   * Save the current settings to the parameter set with the given name.
   *
   * If the projector or front light have been enabled or disabled manually,
   * the flag should be set. It then disabled the automatic control of the
   * projector and front light for this parameter set.
   */
  void saveParameterSet(std::string name, bool projectorWritten = false);

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
  ros::Time capture() const;

  /**
   * Try to collect patterns on the current images. For the command to be
   * successful, the patterns must be decodable and visible in both cameras.
   */
  std::vector<CalibrationPattern> collectPattern(bool clearBuffer = false) const;

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
  tf::Stamped<tf::Pose> estimatePatternPose(ros::Time imageTimestamp = ros::Time::now(),
                                            std::string const& targetFrame = "",
                                            bool latestPatternOnly = false) const;

  /**
   * Estimate the pose of each pattern in the pattern buffer.
   */
  std::vector<tf::Stamped<tf::Pose>> estimatePatternPoses(ros::Time imageTimestamp = ros::Time::now(),
                                                          std::string const& targetFrame = "") const;

  /**
   * Update the camera's link node and the transformations in the NxLib
   * according to the current information from TF.
   *
   * @param time                    The timestamp from which the transformation should be taken.
   * @param targetFrame             The TF frame in which the camera should return the data. Uses the node's target
   *                                frame by default.
   * @param useCachedTransformation Do not update the transformation from the TF server, but use a cached one.
   */
  void updateTransformations(ros::Time time = ros::Time::now(), std::string targetFrame = "",
                             bool useCachedTransformation = false) const;

  /**
   * Update the camera's link node and the transformations in the NxLib
   * to the given transformation. The given transformation should take data
   * from the camera frame to some target frame.
   */
  void updateTransformations(tf::Pose const& targetFrameTransformation) const;

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
  void updateCameraInfo();

  ensenso_camera_msgs::ParameterPtr readParameter(std::string const& key) const;
  void writeParameter(ensenso_camera_msgs::Parameter const& parameter);
};
