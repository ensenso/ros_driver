#pragma once

#include "ensenso_camera_msgs/AccessTreeAction.h"
#include "ensenso_camera_msgs/CalibrateHandEyeAction.h"
#include "ensenso_camera_msgs/CalibrateWorkspaceAction.h"
#include "ensenso_camera_msgs/ExecuteCommandAction.h"
#include "ensenso_camera_msgs/FitPrimitiveAction.h"
#include "ensenso_camera_msgs/GetParameterAction.h"
#include "ensenso_camera_msgs/SetParameterAction.h"
#include "ensenso_camera/calibration_pattern.h"
#include "ensenso_camera/image_utilities.h"
#include "ensenso_camera/nxlib_version.h"
#include "ensenso_camera/point_cloud_utilities.h"
#include "ensenso_camera/queued_action_server.h"
#include "ensenso_camera/string_helper.h"
#include "ensenso_camera/virtual_object_handler.h"

#include "nxLib.h"

#include <tf2/LinearMath/Transform.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <fstream>

/**
 * The interval at which we publish diagnostic messages containing the camera status.
 */
double const STATUS_INTERVAL = 3.0;  // Seconds.

/**
 * The interval at which we publish the current tf transform from the camera to the target frame, if a target frame is
 * available.
 */
double const POSE_TF_INTERVAL = 1;  // Seconds.

/**
 * The maximum time that we wait for a tf transformation to become available.
 */
double const TF_REQUEST_TIMEOUT = 10.;  // Seconds.

/**
 * The name of the parameter set that is used when an action was not given a parameter set explicitly.
 */
std::string const DEFAULT_PARAMETER_SET = "default";

/**
 * The name of the target frame in the NxLib.
 */
std::string const TARGET_FRAME_LINK = "Workspace";

// The ROS node gives back error codes from the NxLib. Additionally, we use the following error codes to indicate errors
// from the ROS node itself.
int const ERROR_CODE_UNKNOWN_EXCEPTION = 100;
int const ERROR_CODE_TF = 101;

#define LOG_NXLIB_EXCEPTION(EXCEPTION)                                                                                 \
  try                                                                                                                  \
  {                                                                                                                    \
    if (EXCEPTION.getErrorCode() == NxLibExecutionFailed)                                                              \
    {                                                                                                                  \
      NxLibItem executionNode(EXCEPTION.getItemPath());                                                                \
      ROS_ERROR("%s: %s", executionNode[itmResult][itmErrorSymbol].asString().c_str(),                                 \
                executionNode[itmResult][itmErrorText].asString().c_str());                                            \
    } /* NOLINT */                                                                                                     \
  }   /* NOLINT */                                                                                                     \
  catch (...)                                                                                                          \
  {                                                                                                                    \
  } /* NOLINT */                                                                                                       \
  ROS_DEBUG("Current NxLib tree: %s", NxLibItem().asJson(true).c_str());

// The following macros are called at the beginning and end of each action handler that uses the NxLib. In case of an
// NxLib exception they automatically abort the action and return the corresponding error code and message.
// This assumes that all of our actions have the property error that represents an NxLibException.

#define START_NXLIB_ACTION(ACTION_NAME, ACTION_SERVER)                                                                 \
  ROS_DEBUG("Received a " #ACTION_NAME " request.");                                                                   \
  auto& server = ACTION_SERVER;                                                                                        \
  if (server->isPreemptRequested())                                                                                    \
  {                                                                                                                    \
    server->setPreempted();                                                                                            \
    return;                                                                                                            \
  } /* NOLINT */                                                                                                       \
  std::lock_guard<std::mutex> lock(nxLibMutex);                                                                        \
  try                                                                                                                  \
  {
#define FINISH_NXLIB_ACTION(ACTION_NAME)                                                                               \
  } /* NOLINT */                                                                                                       \
  catch (NxLibException & e)                                                                                           \
  {                                                                                                                    \
    ROS_ERROR("NxLibException %d (%s) for item %s", e.getErrorCode(), e.getErrorText().c_str(),                        \
              e.getItemPath().c_str());                                                                                \
    LOG_NXLIB_EXCEPTION(e)                                                                                             \
    ensenso_camera_msgs::ACTION_NAME##Result result;                                                                   \
    result.error.code = e.getErrorCode();                                                                              \
    result.error.message = e.getErrorText();                                                                           \
    server->setAborted(std::move(result));                                                                             \
    return;                                                                                                            \
  } /* NOLINT */                                                                                                       \
  catch (tf2::TransformException & e)                                                                                  \
  {                                                                                                                    \
    ROS_ERROR("tf Exception: %s", e.what());                                                                           \
    ensenso_camera_msgs::ACTION_NAME##Result result;                                                                   \
    result.error.code = ERROR_CODE_TF;                                                                                 \
    result.error.message = e.what();                                                                                   \
    server->setAborted(std::move(result));                                                                             \
    return;                                                                                                            \
  } /* NOLINT */                                                                                                       \
  catch (std::exception & e)                                                                                           \
  {                                                                                                                    \
    ROS_ERROR("Unknown Exception: %s", e.what());                                                                      \
    ensenso_camera_msgs::ACTION_NAME##Result result;                                                                   \
    result.error.code = ERROR_CODE_UNKNOWN_EXCEPTION;                                                                  \
    result.error.message = e.what();                                                                                   \
    server->setAborted(std::move(result));                                                                             \
    return;                                                                                                            \
  }

#define PREEMPT_ACTION_IF_REQUESTED                                                                                    \
  if (server->isPreemptRequested())                                                                                    \
  {                                                                                                                    \
    server->setPreempted();                                                                                            \
    return;                                                                                                            \
  }

using AccessTreeServer = QueuedActionServer<ensenso_camera_msgs::AccessTreeAction>;
using ExecuteCommandServer = QueuedActionServer<ensenso_camera_msgs::ExecuteCommandAction>;
using GetParameterServer = QueuedActionServer<ensenso_camera_msgs::GetParameterAction>;
using SetParameterServer = QueuedActionServer<ensenso_camera_msgs::SetParameterAction>;

/**
 * A set of parameters that can be used by the different actions of the camera node.
 */
struct ParameterSet
{
  /**
   * An NxLib node in which we store the NxLib camera parameters for this parameter set.
   */
  NxLibItem node;

  /**
   * Whether the 3D region of interest is enabled for this parameter set.
   */
  bool useROI = false;

  /**
   * A 3D region of interest by which the point cloud is filtered before it is published.
   */
  PointCloudROI roi;

  /**
   * Whether projector and front light are managed automatically for this parameter set. This gets disabled as soon as
   * either the projector or the front light are set manually.
   */
  bool autoProjector = true;

  ParameterSet(std::string const& name, NxLibItem const& defaultParameters);
};

/**
 * The camera parameters that can be used for different camera types (Monocular, Stereo and StructuredLight).
 */
struct CameraParameters
{
  /**
   * The camera serial.
   */
  std::string serial;

  /**
   * Whether the camera is a file camera.
   *
   */
  bool isFileCamera;

  /**
   * The path to the data of the file camera.
   */
  std::string fileCameraPath;

  /**
   * Whether the camera is fixed in the world or moves with a robot.
   */
  bool fixed;

  /**
   * Whether the node should wait for the camera to become available.
   */
  bool wait_for_camera = false;

  /**
   * The tf frame in which the data is captured by the camera.
   */
  std::string cameraFrame;

  /**
   * A helper tf frame.
   *
   * If the linkFrame parameter is not given, it defaults to targetFrame if targetFrame is given, otherwise linkFrame
   * will be the same as the cameraFrame.
   * If the linkFrame parameter is given, it represents the internal camera link, which is stored for each of the
   * cameras in the NxLib. This camera link stores e.g. the transformation of the hand-eye calibration (fixed) from the
   * camera to the robot base. It can also store a transformation to another camera or the transformation to the
   * workspace, which is received by a workspace calibration. This internal camera link is static and will be published
   * with tf.
   */
  std::string linkFrame;

  /**
   * The tf frame in which the user wants to receive the data from the camera. All data is automatically transformed
   * using the current tf transformation between linkFrame and targetFrame (this includes the point cloud as well as
   * poses of calibration patterns). By default, targetFrame is the same as cameraFrame, if not defined.
   */
  std::string targetFrame;

  /**
   * Optional tf frame of the robot's base for a hand-eye calibration of a stereo camera. For a fixed camera, this
   * defaults to cameraFrame, for a moving camera it needs to be specified if you want to perform a hand-eye
   * calibration.
   */
  std::string robotFrame = "";

  /**
   * Optional tf frame of the robot's wrist for a hand-eye calibration of a stereo camera. For a moving camera, this
   * defaults to cameraFrame, for a fixed camera it needs to be specified if you want to perform a hand-eye calibration.
   */
  std::string wristFrame = "";

  /**
   * Optional capture timeout for a stereo camera.
   */
  int captureTimeout = 0;

  /**
   * Optional VirtualObjectHandler for a stereo camera.
   */
  std::unique_ptr<ensenso_camera::VirtualObjectHandler> virtualObjectHandler = nullptr;

  CameraParameters(ros::NodeHandle const& nh, std::string const& cameraType, std::string serial);
};

class Camera
{
protected:
  CameraParameters params;

  // Whether the camera is a file camera and was created by this node. If it was/ created by this node, it will be
  // deleted after it got closed.
  bool createdFileCamera = false;

  NxLibVersion nxLibVersion;

  NxLibItem cameraNode;

  // Controls parallel access to the NxLib.
  mutable std::mutex nxLibMutex;

  ros::NodeHandle nh;

  ros::Publisher statusPublisher;
  ros::Timer statusTimer;
  ros::Timer cameraPosePublisher;

  // tf buffer, that will store transformations for 10 seconds and dropping transformation afterwards.
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> transformListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;

  std::unique_ptr<AccessTreeServer> accessTreeServer;
  std::unique_ptr<ExecuteCommandServer> executeCommandServer;
  std::unique_ptr<GetParameterServer> getParameterServer;
  std::unique_ptr<SetParameterServer> setParameterServer;

  // Saves the latest transforms for a specific frame.
  mutable std::map<std::string, geometry_msgs::TransformStamped> transformationCache;

  // Contains a parameter tree that is used for creating new parameter sets.
  NxLibItem defaultParameters;

  std::map<std::string, ParameterSet> parameterSets;
  std::string currentParameterSet;

public:
  Camera(ros::NodeHandle& nh, CameraParameters params);

  /**
   * Open the camera.
   */
  bool open();

  /**
   * Update camera type specifics (used for the S-series, which is a subtype of stereo).
   */
  virtual void updateCameraTypeSpecifics()
  {
  }

  /**
   * Initialize the camera.
   */
  virtual void init() = 0;

  /**
   * Close the camera.
   */
  void close();

  /**
   * Start the action servers. The camera must already be opened, otherwise the actions might access parts of the NxLib
   * that are not initialized yet.
   */
  virtual void startServers() const;

  /**
   * Load the camera settings from the given JSON file. The resulting parameters will also be saved as the default
   * values for new parameter sets.
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
  virtual void onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal) = 0;

protected:
  /**
   * Start publishing the camera links to tf
   */
  virtual void initTfPublishTimer();

  /**
   * Initialize the status Timer
   */
  virtual void initStatusTimer();

  /**
   * Save the current settings to the parameter set with the given name.
   *
   * If the projector or front light have been enabled or disabled manually, the flag should be set. It then disabled
   * the automatic control of the projector and front light for this parameter set.
   */
  void saveParameterSet(std::string name);

  /**
   * Check whether the camera is available for opening.
   */
  bool cameraIsAvailable() const;

  /**
   * Check whether the camera is still open in the NxLib.
   */
  bool cameraIsOpen() const;

  /**
   * Check whether the camera has an internal link.
   */
  bool hasLink() const;

  /**
   * Publish a diagnostic message indicating whether the camera is still open and usable in the NxLib.
   */
  virtual void publishStatus(ros::TimerEvent const& event) const;

  /**
   * Read the current parameters from the camera node and store them as the default parameter set that will later be
   * used for creating new parameter sets.
   */
  void saveDefaultParameterSet();
  /**
   * Load the parameter set with the given name. If it does not exist yet, it will be created by copying the current
   * default parameters.
   */
  void loadParameterSet(std::string name);

  /**
   * Capture a new pair of images. Returns the timestamp of the (first) captured image.
   */
  virtual ros::Time capture() const = 0;

  /**
   * Estimate the pose of a pattern in the given tf frame. The pattern must already be contained in the pattern buffer
   * (that is, you should call collectPattern before this function).
   *
   * When the latestPatternOnly flag is set, the estimated position will be the one of the latest pattern in the buffer.
   * Otherwise the function assumes that all observations are of the same pattern. It will then average their positions
   * to increase the accuracy of the pose estimation.
   */
  virtual geometry_msgs::PoseStamped estimatePatternPose(ros::Time imageTimestamp = ros::Time::now(),
                                                         std::string const& targetFrame = "",
                                                         bool latestPatternOnly = false) const = 0;

  /**
   * Estimate the pose of each pattern in the pattern buffer.
   */
  virtual std::vector<geometry_msgs::PoseStamped> estimatePatternPoses(ros::Time imageTimestamp = ros::Time::now(),
                                                                       std::string const& targetFrame = "") const = 0;

  /**
   * Update the camera's link node and the transformations in the NxLib according to the current information from tf.
   *
   * The target frame is node's target frame by default. When the useCachedTransformation flag is set, the
   * transformation is not updated from the tf server and a cached tranformation is used instead.
   */
  void updateGlobalLink(ros::Time time = ros::Time::now(), std::string targetFrame = "",
                        bool useCachedTransformation = false) const;

  /**
   * Return the target frame name for links in the NxLib.
   */
  std::string getNxLibTargetFrameName() const;

  /**
   * Update the camera's link node and the transformations in the NxLib to the given transformation. The given
   * transformation should take data from the camera frame to some target frame.
   */
  void updateTransformations(tf2::Transform const& targetFrameTransformation) const;

  /**
   * Read the camera calibration from the NxLib and write it into a CameraInfo message.
   */
  void fillBasicCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info) const;

  /**
   * Update the cached CameraInfo messages that will be published together with the images.
   */
  virtual void updateCameraInfo() = 0;

  /**
   * Read the parameter with the given key from the NxLib tree.
   */
  virtual ensenso_camera_msgs::ParameterPtr readParameter(std::string const& key) const;

  /**
   * Write the given parameter to the NxLib tree.
   */
  virtual void writeParameter(ensenso_camera_msgs::Parameter const& parameter);

  /**
   * Callback wrapper for publishCameraLink(). Can be used e.g. with ros::NodeHandle::createTimer().
   *
   * The timerEvent parameter defines the rate with which the transformations are getting published.
   */
  void publishCurrentLinks(ros::TimerEvent const& timerEvent = ros::TimerEvent());

  /**
   * Publish both the internal calibrated link and, if existing, the global link from the camera to the global frame.
   */
  void publishCameraLink();

  /**
   * Create a stamped transform message from link to camera frame.
   */
  geometry_msgs::TransformStamped stampedLinkToCamera();

  /**
   * Create a transform from camera to link frame.
   */
  tf2::Transform getCameraToLinkTransform();
};
