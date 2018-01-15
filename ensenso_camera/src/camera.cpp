#include "ensenso_camera/camera.h"

#include <cmath>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/distortion_models.h>
#include "pcl_ros/point_cloud.h"

#include "ensenso_camera/helper.h"
#include "ensenso_camera/image_utilities.h"
#include "ensenso_camera/parameters.h"
#include "ensenso_camera/pose_utilities.h"

/**
 * The interval at which we publish diagnostic messages containing the camera
 * status.
 */
double const STATUS_INTERVAL = 3.0;  // Seconds.

/**
 * The maximum time that we wait for a tf transformation to become available.
 */
double const TRANSFORMATION_REQUEST_TIMEOUT = 0.1;  // Seconds.

/**
 * The name of the parameter set that is used when an action was not given a
 * parameter set explicitly.
 */
std::string const DEFAULT_PARAMETER_SET = "default";

/**
 * The name of the target frame in the NxLib.
 */
std::string const TARGET_FRAME_LINK = "Workspace";

// The ROS node gives back error codes from the NxLib. Additionally, we use the
// following error codes to indicate errors from the ROS node itself.
int const ERROR_CODE_UNKNOWN_EXCEPTION = 100;
int const ERROR_CODE_TF = 101;

#define LOG_NXLIB_EXCEPTION(EXCEPTION) \
  try \
  { \
    if (EXCEPTION.getErrorCode() == NxLibExecutionFailed) \
    { \
      NxLibItem executionNode(EXCEPTION.getItemPath()); \
      ROS_ERROR("%s: %s", executionNode[itmResult][itmErrorSymbol].asString().c_str(), \
                executionNode[itmResult][itmErrorText].asString().c_str()); \
    } /* NOLINT */ \
  } catch (...) {} /* NOLINT */ \
  ROS_DEBUG("Current NxLib tree: %s", NxLibItem().asJson(true).c_str()); \

// The following macros are called at the beginning and end of each action
// handler that uses the NxLib. In case of an NxLib exception they
// automatically abort the action and return the corresponding error code and
// message.
// This assumes that all of our actions have the property error that represents
// an NxLibException.

#define START_NXLIB_ACTION(ACTION_NAME, ACTION_SERVER) \
  ROS_DEBUG("Received a " #ACTION_NAME " request."); \
  auto& server = ACTION_SERVER; \
  if (server->isPreemptRequested()) \
  { \
    server->setPreempted(); \
    return; \
  } /* NOLINT */ \
  std::lock_guard<std::mutex> lock(nxLibMutex); \
  try \
  {
#define FINISH_NXLIB_ACTION(ACTION_NAME) \
  } /* NOLINT */ \
  catch (NxLibException& e) \
  { \
    ROS_ERROR("NxLibException %d (%s) for item %s", e.getErrorCode(), e.getErrorText().c_str(), \
              e.getItemPath().c_str()); \
    LOG_NXLIB_EXCEPTION(e) \
    ensenso_camera_msgs::ACTION_NAME##Result result; \
    result.error.code = e.getErrorCode(); \
    result.error.message = e.getErrorText(); \
    server->setAborted(result); \
    return; \
  } /* NOLINT */ \
  catch (tf::TransformException& e) \
  { \
    ROS_ERROR("TF Exception: %s", e.what()); \
    ensenso_camera_msgs::ACTION_NAME##Result result; \
    result.error.code = ERROR_CODE_TF; \
    result.error.message = e.what(); \
    server->setAborted(result); \
    return; \
  } /* NOLINT */ \
  catch (std::exception& e) \
  { \
    ROS_ERROR("Unknown Exception: %s", e.what()); \
    ensenso_camera_msgs::ACTION_NAME##Result result; \
    result.error.code = ERROR_CODE_UNKNOWN_EXCEPTION; \
    result.error.message = e.what(); \
    server->setAborted(result); \
    return; \
  }

#define PREEMPT_ACTION_IF_REQUESTED \
  if (server->isPreemptRequested()) \
  { \
    server->setPreempted(); \
    return; \
  }

/**
 * Check whether the NxLib has at least the given version.
 */
bool checkNxLibVersion(int major, int minor)
{
    int nxLibMajor = NxLibItem()[itmVersion][itmMajor].asInt();
    int nxLibMinor = NxLibItem()[itmVersion][itmMinor].asInt();
    return (nxLibMajor > major) || (nxLibMajor == major && nxLibMinor >= minor);
}

ParameterSet::ParameterSet(const std::string &name, const NxLibItem &defaultParameters)
{
  // Create a new NxLib node where we will store the parameters for this set and
  // overwrite it with the default settings.
  node = NxLibItem()["rosParameterSets"][name];
  node << defaultParameters;
}

Camera::Camera(ros::NodeHandle nh, std::string const& serial, std::string const& fileCameraPath, bool fixed,
               std::string const& cameraFrame, std::string const& targetFrame, std::string const& robotFrame,
               std::string const& wristFrame)
  : serial(serial)
  , fileCameraPath(fileCameraPath)
  , fixed(fixed)
  , cameraFrame(cameraFrame)
  , targetFrame(targetFrame)
  , robotFrame(robotFrame)
  , wristFrame(wristFrame)
{
  isFileCamera = !fileCameraPath.empty();

  leftCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  rightCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  leftRectifiedCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  rightRectifiedCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();

  accessTreeServer =
      make_unique<AccessTreeServer>(nh, "access_tree", boost::bind(&Camera::onAccessTree, this, _1));
  executeCommandServer =
      make_unique<ExecuteCommandServer>(nh, "execute_command", boost::bind(&Camera::onExecuteCommand, this, _1));

  getParameterServer =
      make_unique<GetParameterServer>(nh, "get_parameter", boost::bind(&Camera::onGetParameter, this, _1));
  setParameterServer =
      make_unique<SetParameterServer>(nh, "set_parameter", boost::bind(&Camera::onSetParameter, this, _1));

  requestDataServer =
      make_unique<RequestDataServer>(nh, "request_data", boost::bind(&Camera::onRequestData, this, _1));
  locatePatternServer =
      make_unique<LocatePatternServer>(nh, "locate_pattern", boost::bind(&Camera::onLocatePattern, this, _1));
  projectPatternServer =
      make_unique<ProjectPatternServer>(nh, "project_pattern", boost::bind(&Camera::onProjectPattern, this, _1));
  calibrateHandEyeServer =
      make_unique<CalibrateHandEyeServer>(nh, "calibrate_hand_eye",
                                          boost::bind(&Camera::onCalibrateHandEye, this, _1));
  calibrateWorkspaceServer =
      make_unique<CalibrateWorkspaceServer>(nh, "calibrate_workspace",
                                            boost::bind(&Camera::onCalibrateWorkspace, this, _1));

  image_transport::ImageTransport imageTransport(nh);

  leftRawImagePublisher = imageTransport.advertiseCamera("raw/left/image", 1);
  rightRawImagePublisher = imageTransport.advertiseCamera("raw/right/image", 1);
  leftRectifiedImagePublisher = imageTransport.advertiseCamera("rectified/left/image", 1);
  rightRectifiedImagePublisher = imageTransport.advertiseCamera("rectified/right/image", 1);
  disparityMapPublisher = imageTransport.advertise("disparity_map", 1);

  pointCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1);

  statusPublisher = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  statusTimer = nh.createTimer(ros::Duration(STATUS_INTERVAL), &Camera::publishStatus, this);

  cameraNode = NxLibItem()[itmCameras][itmBySerialNo][serial];

  defaultParameters = NxLibItem()["rosDefaultParameters"];
}

bool Camera::open()
{
  if (isFileCamera && !cameraNode.exists())
  {
    try
    {
      if (serial.size() > 15)
      {
        ROS_ERROR("The serial '%s' is too long!", serial.c_str());
        return false;
      }

      NxLibCommand createCamera(cmdCreateCamera);
      createCamera.parameters()[itmSerialNumber] = serial;
      createCamera.parameters()[itmFolderPath] = fileCameraPath;
      createCamera.execute();

      createdFileCamera = true;
    }
    catch (NxLibException& e)
    {
      ROS_ERROR("Failed to create the file camera!");
      LOG_NXLIB_EXCEPTION(e)
      return false;
    }
  }

  if (!cameraNode.exists())
  {
    ROS_ERROR("The camera '%s' could not be found", serial.c_str());
    return false;
  }
  if (!cameraIsAvailable())
  {
    ROS_ERROR("The camera '%s' is already in use", serial.c_str());
    return false;
  }

  try
  {
    NxLibCommand open(cmdOpen);
    open.parameters()[itmCameras] = serial;
    open.execute();
  }
  catch (NxLibException& e)
  {
    ROS_ERROR("Error while opening the camera '%s'!", serial.c_str());
    LOG_NXLIB_EXCEPTION(e)
    return false;
  }

  ROS_INFO("Opened camera with serial number '%s'.", serial.c_str());

  updateCameraInfo();
  saveDefaultParameterSet();

  return true;
}

void Camera::close()
{
  std::lock_guard<std::mutex> lock(nxLibMutex);

  try
  {
    NxLibCommand close(cmdClose);
    close.parameters()[itmCameras] = serial;
    close.execute();

    if (isFileCamera && createdFileCamera)
    {
      NxLibCommand deleteCmd(cmdDeleteCamera);
      deleteCmd.parameters()[itmCameras] = serial;
      deleteCmd.execute();
    }
  }
  catch (NxLibException&)
  {
  }
}

void Camera::startServers() const
{
  accessTreeServer->start();
  executeCommandServer->start();

  getParameterServer->start();
  setParameterServer->start();

  requestDataServer->start();
  locatePatternServer->start();
  projectPatternServer->start();
  calibrateHandEyeServer->start();
  calibrateWorkspaceServer->start();
}

bool Camera::loadSettings(const std::string& jsonFile, bool saveAsDefaultParameters)
{
  if (jsonFile.empty()) return true;

  std::ifstream file(jsonFile);
  if (file.is_open() && file.rdbuf())
  {
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string const& jsonSettings = buffer.str();

    NxLibItem tmpParameters = NxLibItem()["rosTemporaryParameters"];
    try
    {
      tmpParameters.setJson(jsonSettings);
    }
    catch (NxLibException&)
    {
      ROS_ERROR("The file '%s' does not contain valid JSON", jsonFile.c_str());
      return false;
    }

    try
    {
      if (tmpParameters[itmParameters].exists())
      {
        // The file contains the entire camera node.
        cameraNode[itmParameters] << tmpParameters[itmParameters];
      }
      else
      {
        // The file contains only the parameter node.
        cameraNode[itmParameters].setJson(jsonSettings, true);
      }
      tmpParameters.erase();

      NxLibCommand synchronize(cmdSynchronize);
      synchronize.parameters()[itmCameras] = serial;
      synchronize.execute();

      updateCameraInfo();
      if (saveAsDefaultParameters) saveDefaultParameterSet();
    }
    catch (NxLibException& e)
    {
      LOG_NXLIB_EXCEPTION(e)
      return false;
    }
  }
  else
  {
    ROS_ERROR("Could not open the file '%s'", jsonFile.c_str());
    return false;
  }

  return true;
}

void Camera::onAccessTree(const ensenso_camera_msgs::AccessTreeGoalConstPtr& goal)
{
  START_NXLIB_ACTION(AccessTree, accessTreeServer)

  NxLibItem item(goal->path);

  if (goal->erase)
  {
    item.erase();
  }
  else if (goal->set_null)
  {
    item.setNull();
  }
  else if (!goal->json_value.empty())
  {
    item.setJson(goal->json_value);
  }

  ensenso_camera_msgs::AccessTreeResult result;

  result.exists = false;
  if (item.exists())
  {
    result.exists = true;

    result.json_value = item.asJson();
    try
    {
      // This could load any image that does not belong to the node's camera,
      // so we do not know its frame.
      result.binary_data = *imageFromNxLibNode(item, "");
    }
    catch (NxLibException&)
    {
    }  // The item was not binary.
  }

  accessTreeServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(AccessTree)
}

void Camera::onExecuteCommand(const ensenso_camera_msgs::ExecuteCommandGoalConstPtr& goal)
{
  START_NXLIB_ACTION(ExecuteCommand, executeCommandServer)

  NxLibCommand command(goal->command);
  command.parameters().setJson(goal->parameters);
  command.execute();

  ensenso_camera_msgs::ExecuteCommandResult result;
  result.result = command.result().asJson();

  executeCommandServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(ExecuteCommand)
}

void Camera::onGetParameter(ensenso_camera_msgs::GetParameterGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(GetParameter, getParameterServer)

  ensenso_camera_msgs::GetParameterResult result;

  loadParameterSet(goal->parameter_set);

  result.stamp = ros::Time::now();
  for (auto const& key : goal->keys)
  {
    result.results.push_back(*readParameter(key));
  }

  getParameterServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(GetParameter)
}

void Camera::onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(SetParameter, setParameterServer)

  ensenso_camera_msgs::SetParameterResult result;

  loadParameterSet(goal->parameter_set);

  result.parameter_file_applied = false;
  if (!goal->parameter_file.empty())
  {
    result.parameter_file_applied = loadSettings(goal->parameter_file);
    if (!result.parameter_file_applied)
    {
      server->setAborted(result);
      return;
    }
  }

  bool projectorChanged = false;
  for (auto const& parameter : goal->parameters)
  {
    writeParameter(parameter);

    if (parameter.key == parameter.PROJECTOR || parameter.key == parameter.FRONT_LIGHT)
    {
      projectorChanged = true;
    }
  }

  // Synchronize to make sure that we read back the correct values.
  NxLibCommand synchronize(cmdSynchronize);
  synchronize.parameters()[itmCameras] = serial;
  synchronize.execute();

  saveParameterSet(goal->parameter_set, projectorChanged);

  // The new parameters might change the camera calibration.
  updateCameraInfo();

  // Read back the actual values.
  for (auto const& parameter : goal->parameters)
  {
    result.results.push_back(*readParameter(parameter.key));
  }

  setParameterServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(SetParameter)
}

void Camera::onRequestData(ensenso_camera_msgs::RequestDataGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(RequestData, requestDataServer)

  ensenso_camera_msgs::RequestDataResult result;
  ensenso_camera_msgs::RequestDataFeedback feedback;

  bool publishResults = goal->publish_results;
  if (!goal->publish_results && !goal->include_results_in_response)
  {
    publishResults = true;
  }

  bool requestPointCloud = goal->request_point_cloud;
  if (!goal->request_raw_images && !goal->request_rectified_images && !goal->request_point_cloud &&
      !goal->request_normals)
  {
    requestPointCloud = true;
  }

  bool computePointCloud = requestPointCloud || goal->request_normals;
  bool computeDisparityMap = goal->request_disparity_map || computePointCloud;

  loadParameterSet(goal->parameter_set, computeDisparityMap ? projectorOn : projectorOff);
  ros::Time imageTimestamp = capture();

  PREEMPT_ACTION_IF_REQUESTED

  feedback.images_acquired = true;
  requestDataServer->publishFeedback(feedback);

  if (goal->request_raw_images)
  {
    auto rawImages = imagesFromNxLibNode(cameraNode[itmImages][itmRaw], cameraFrame);

    leftCameraInfo->header.stamp = rawImages[0].first->header.stamp;
    rightCameraInfo->header.stamp = leftCameraInfo->header.stamp;

    if (goal->include_results_in_response)
    {
      for (auto const& imagePair : rawImages)
      {
        result.left_raw_images.push_back(*imagePair.first);
        result.right_raw_images.push_back(*imagePair.second);
      }
      result.left_camera_info = *leftCameraInfo;
      result.right_camera_info = *rightCameraInfo;
    }
    if (publishResults)
    {
      // We only publish one of the images on the topic, even if FlexView is enabled.
      leftRawImagePublisher.publish(rawImages[0].first, leftCameraInfo);
      rightRawImagePublisher.publish(rawImages[0].second, rightCameraInfo);
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  // If we need the disparity map, we do the rectification implicitly in cmdComputeDisparityMap. This is more
  // efficient when using CUDA.
  if (goal->request_rectified_images && !computeDisparityMap)
  {
    NxLibCommand rectify(cmdRectifyImages);
    rectify.parameters()[itmCameras] = serial;
    rectify.execute();
  }
  else if (computeDisparityMap)
  {
    NxLibCommand computeDisparityMap(cmdComputeDisparityMap);
    computeDisparityMap.parameters()[itmCameras] = serial;
    computeDisparityMap.execute();
  }

  if (goal->request_rectified_images)
  {
    auto rectifiedImages = imagesFromNxLibNode(cameraNode[itmImages][itmRectified], cameraFrame);

    leftRectifiedCameraInfo->header.stamp = rectifiedImages[0].first->header.stamp;
    rightRectifiedCameraInfo->header.stamp = leftRectifiedCameraInfo->header.stamp;

    if (goal->include_results_in_response)
    {
      for (auto const& imagePair : rectifiedImages)
      {
        result.left_rectified_images.push_back(*imagePair.first);
        result.right_rectified_images.push_back(*imagePair.second);
      }
      result.left_rectified_camera_info = *leftRectifiedCameraInfo;
      result.right_rectified_camera_info = *rightRectifiedCameraInfo;
    }
    if (publishResults)
    {
      // We only publish one of the images on the topic, even if FlexView is enabled.
      leftRectifiedImagePublisher.publish(rectifiedImages[0].first, leftRectifiedCameraInfo);
      rightRectifiedImagePublisher.publish(rectifiedImages[0].second, rightRectifiedCameraInfo);
    }
  }

  if (goal->request_disparity_map)
  {
    auto disparityMap = imageFromNxLibNode(cameraNode[itmImages][itmDisparityMap], cameraFrame);

    if (goal->include_results_in_response)
    {
      result.disparity_map = *disparityMap;
    }
    if (publishResults)
    {
      disparityMapPublisher.publish(disparityMap);
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  PointCloudROI const* pointCloudROI = 0;
  if (parameterSets.at(currentParameterSet).useROI)
  {
    pointCloudROI = &parameterSets.at(currentParameterSet).roi;
  }

  if (computePointCloud)
  {
    updateTransformations(imageTimestamp, "", goal->use_cached_transformation);

    NxLibCommand computePointMap(cmdComputePointMap);
    computePointMap.parameters()[itmCameras] = serial;
    computePointMap.execute();

    if (requestPointCloud && !goal->request_normals)
    {
      auto pointCloud = pointCloudFromNxLib(cameraNode[itmImages][itmPointMap], targetFrame, pointCloudROI);

      if (goal->include_results_in_response)
      {
        pcl::toROSMsg(*pointCloud, result.point_cloud);
      }
      if (publishResults)
      {
        pointCloudPublisher.publish(pointCloud);
      }
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  if (goal->request_normals)
  {
    NxLibCommand(cmdComputeNormals).execute();

    auto pointCloud = pointCloudWithNormalsFromNxLib(cameraNode[itmImages][itmPointMap],
                                                     cameraNode[itmImages][itmNormals], targetFrame, pointCloudROI);

    if (goal->include_results_in_response)
    {
      pcl::toROSMsg(*pointCloud, result.point_cloud);
    }
    if (publishResults)
    {
      pointCloudPublisher.publish(pointCloud);
    }
  }

  requestDataServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(RequestData)
}

void Camera::onLocatePattern(ensenso_camera_msgs::LocatePatternGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(LocatePattern, locatePatternServer)

  ensenso_camera_msgs::LocatePatternResult result;
  ensenso_camera_msgs::LocatePatternFeedback feedback;

  loadParameterSet(goal->parameter_set, projectorOff);

  int numberOfShots = goal->number_of_shots;
  if (numberOfShots < 1)
  {
    numberOfShots = 1;
  }

  std::vector<CalibrationPattern> patterns;
  ros::Time imageTimestamp;
  for (int i = 0; i < numberOfShots; i++)
  {
    PREEMPT_ACTION_IF_REQUESTED

    ros::Time timestamp = capture();
    if (i == 0) imageTimestamp = timestamp;

    PREEMPT_ACTION_IF_REQUESTED

    if (i == (numberOfShots - 1))
    {
      // The last image has been captured.
      feedback.images_acquired = true;
      locatePatternServer->publishFeedback(feedback);
    }

    bool clearBuffer = (i == 0);
    patterns = collectPattern(clearBuffer);
    if (patterns.empty())
    {
      result.found_pattern = false;
      locatePatternServer->setSucceeded(result);
      return;
    }

    if (patterns.size() > 1)
    {
      // Cannot average multiple shots of multiple patterns. We will cancel the
      // capturing and estimate the pose of each pattern individually.
      break;
    }
  }

  result.found_pattern = true;
  result.patterns.resize(patterns.size());
  for (size_t i = 0; i < patterns.size(); i++)
  {
    patterns[i].writeToMessage(&result.patterns[i]);
  }

  PREEMPT_ACTION_IF_REQUESTED

  std::string patternFrame = targetFrame;
  if (!goal->target_frame.empty())
  {
    patternFrame = goal->target_frame;
  }

  result.frame = patternFrame;

  if (patterns.size() > 1)
  {
    // Estimate the pose of all the patterns we found.
    auto patternPoses = estimatePatternPoses(imageTimestamp, patternFrame);

    result.pattern_poses.resize(patternPoses.size());
    for (size_t i = 0; i < patternPoses.size(); i++)
    {
      tf::poseStampedTFToMsg(patternPoses[i], result.pattern_poses[i]);
    }
  }
  else
  {
    // Estimate the pose of a single pattern, averaging over the different shots.
    tf::Stamped<tf::Pose> patternPose = estimatePatternPose(imageTimestamp, patternFrame);

    result.pattern_poses.resize(1);
    tf::poseStampedTFToMsg(patternPose, result.pattern_poses[0]);
  }

  if (!goal->tf_frame.empty())
  {
    if (patterns.size() == 1)
    {
      tf::StampedTransform transform = transformFromPose(result.pattern_poses[0], goal->tf_frame);
      transformBroadcaster.sendTransform(transform);
    }
    else
    {
      ROS_WARN("Cannot publish the pattern pose in TF, because there are multiple patterns!");
    }
  }

  locatePatternServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(LocatePattern)
}

void Camera::onProjectPattern(ensenso_camera_msgs::ProjectPatternGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(ProjectPattern, projectPatternServer)

  ensenso_camera_msgs::ProjectPatternResult result;

  tf::Pose targetFrameTransformation;
  tf::poseMsgToTF(goal->target_frame_transformation, targetFrameTransformation);
  if (poseIsValid(targetFrameTransformation))
  {
    updateTransformations(targetFrameTransformation);
  }
  else
  {
    updateTransformations();
  }

  PREEMPT_ACTION_IF_REQUESTED

  if (!checkNxLibVersion(2, 1))
  {
    // In old NxLib versions, the project pattern command does not take the grid spacing as a parameter.
    NxLibItem()[itmParameters][itmPattern][itmGridSpacing] = goal->grid_spacing * 1000;
  }

  tf::Pose patternPose;
  tf::poseMsgToTF(goal->pattern_pose, patternPose);

  NxLibCommand projectPattern(cmdProjectPattern);
  projectPattern.parameters()[itmCameras] = serial;
  projectPattern.parameters()[itmGridSpacing] = goal->grid_spacing * 1000;
  projectPattern.parameters()[itmGridSize][0] = goal->grid_size_x;
  projectPattern.parameters()[itmGridSize][1] = goal->grid_size_y;
  writePoseToNxLib(patternPose, projectPattern.parameters()[itmPatternPose]);
  projectPattern.execute();

  PREEMPT_ACTION_IF_REQUESTED

  int sensorWidth = cameraNode[itmSensor][itmSize][0].asInt();
  int sensorHeight = cameraNode[itmSensor][itmSize][1].asInt();

  NxLibItem projectedPoints = projectPattern.result()[itmStereo][0][itmPoints];
  NxLibItem leftPoints = projectedPoints[0];
  NxLibItem rightPoints = projectedPoints[1];

  result.pattern_is_visible = true;
  result.left_points.resize(leftPoints.count());
  result.right_points.resize(leftPoints.count());
  for (int i = 0; i < leftPoints.count(); i++)
  {
    double leftX = leftPoints[i][0].asDouble();
    double leftY = leftPoints[i][1].asDouble();
    double rightX = rightPoints[i][0].asDouble();
    double rightY = rightPoints[i][1].asDouble();

    result.left_points[i].x = leftX;
    result.left_points[i].y = leftY;
    result.right_points[i].x = rightX;
    result.right_points[i].y = rightY;

    if (leftX < 0 || leftX > sensorWidth || leftY < 0 || leftY > sensorHeight || rightX < 0 || rightX > sensorWidth ||
        rightY < 0 || rightY > sensorHeight)
    {
      result.pattern_is_visible = false;
    }
  }

  projectPatternServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(ProjectPattern)
}

void Camera::onCalibrateHandEye(ensenso_camera_msgs::CalibrateHandEyeGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(CalibrateHandEye, calibrateHandEyeServer)

  ensenso_camera_msgs::CalibrateHandEyeResult result;
  result.command = goal->command;

  if (goal->command == goal->RESET)
  {
    handEyeCalibrationPatternBuffer.clear();
    handEyeCalibrationRobotPoses.clear();
  }
  else if (goal->command == goal->CAPTURE_PATTERN)
  {
    if (robotFrame.empty() || wristFrame.empty())
    {
      result.error_message = "You need to specify a robot base and wrist frame "
                             "to do a hand eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    loadParameterSet(goal->parameter_set, projectorOff);
    ros::Time imageTimestamp = capture();

    PREEMPT_ACTION_IF_REQUESTED

    // Load the pattern buffer that we remembered from the previous
    // calibration steps.
    if (!handEyeCalibrationPatternBuffer.empty())
    {
      NxLibCommand setPatternBuffer(cmdSetPatternBuffer);
      setPatternBuffer.parameters()[itmPatterns] << handEyeCalibrationPatternBuffer;
      setPatternBuffer.execute();
    }
    else
    {
      NxLibCommand(cmdDiscardPatterns).execute();
    }

    std::vector<CalibrationPattern> patterns = collectPattern();
    if (patterns.empty())
    {
      result.found_pattern = false;
      calibrateHandEyeServer->setSucceeded(result);
      return;
    }
    if (patterns.size() > 1)
    {
      result.error_message = "Detected multiple calibration patterns during a hand eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    PREEMPT_ACTION_IF_REQUESTED

    result.found_pattern = true;
    patterns[0].writeToMessage(&result.pattern);

    tf::Pose patternPose = estimatePatternPose(imageTimestamp, cameraFrame, true);
    tf::poseTFToMsg(patternPose, result.pattern_pose);

    PREEMPT_ACTION_IF_REQUESTED

    tf::StampedTransform robotPose;
    try
    {
      transformListener.lookupTransform(robotFrame, wristFrame, ros::Time(0), robotPose);
    }
    catch (tf::TransformException& e)
    {
      result.error_message = std::string("Could not look up the robot pose due to the TF error: ") + e.what();
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    // Remember the newly collected data for the next step.
    NxLibCommand getPatternBuffer(cmdGetPatternBuffer);
    getPatternBuffer.execute();
    handEyeCalibrationPatternBuffer = getPatternBuffer.result()[itmPatterns].asJson();

    handEyeCalibrationRobotPoses.push_back(robotPose);

    tf::poseTFToMsg(robotPose, result.robot_pose);
  }
  else if (goal->command == goal->START_CALIBRATION)
  {
    if (handEyeCalibrationRobotPoses.size() < 5)
    {
      result.error_message = "You need collect at least 5 patterns before starting a hand eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    // Load the pattern observations.
    size_t numberOfPatterns = 0;
    NxLibCommand setPatternBuffer(cmdSetPatternBuffer);
    if (goal->pattern_observations.size() > 0)
    {
      for (size_t i = 0; i < goal->pattern_observations.size(); i++)
      {
        CalibrationPattern pattern(goal->pattern_observations[i]);

        NxLibItem patternNode = setPatternBuffer.parameters()[itmPatterns][i][serial];
        pattern.writeToNxLib(patternNode[itmLeft][0]);
        pattern.writeToNxLib(patternNode[itmRight][0], true);
      }
    }
    else
    {
      setPatternBuffer.parameters()[itmPatterns] << handEyeCalibrationPatternBuffer;
    }
    numberOfPatterns = setPatternBuffer.parameters()[itmPatterns].count();
    setPatternBuffer.execute();

    // Load the corresponding robot poses.
    auto robotPoses = handEyeCalibrationRobotPoses;
    if (goal->robot_poses.size() > 0)
    {
      robotPoses.clear();
      for (auto const& pose : goal->robot_poses)
      {
        tf::Pose tfPose;
        tf::poseMsgToTF(pose, tfPose);
        robotPoses.push_back(tfPose);
      }
    }

    if (robotPoses.size() != numberOfPatterns)
    {
      result.error_message = "The number of pattern observations does not match the number of robot poses!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    // Load the initial guesses from the action goal.
    tf::Pose link, patternPose;
    tf::poseMsgToTF(goal->link, link);
    tf::poseMsgToTF(goal->pattern_pose, patternPose);

    NxLibCommand calibrateHandEye(cmdCalibrateHandEye);
    calibrateHandEye.parameters()[itmSetup] = fixed ? valFixed : valMoving;
    // The target node will be reset anyway before we calculate data for the next time.
    calibrateHandEye.parameters()[itmTarget] = TARGET_FRAME_LINK;
    if (poseIsValid(link))
    {
      writePoseToNxLib(link.inverse(), calibrateHandEye.parameters()[itmLink]);
    }
    if (poseIsValid(patternPose))
    {
      writePoseToNxLib(patternPose, calibrateHandEye.parameters()[itmPatternPose]);
    }
    for (size_t i = 0; i < robotPoses.size(); i++)
    {
      writePoseToNxLib(robotPoses[i], calibrateHandEye.parameters()[itmTransformations][i]);
    }

    calibrateHandEye.execute(false);

    auto getCalibrationResidual = [](NxLibItem const &node)
    {
      if (node[itmResidual].exists())
      {
        return node[itmResidual].asDouble();
      }
      // Compatibility with the SDK 2.0.
      return node[itmReprojectionError].asDouble();
    };

    ros::Rate waitingRate(5);
    while (!calibrateHandEye.finished())
    {
      ensenso_camera_msgs::CalibrateHandEyeFeedback feedback;
      feedback.number_of_iterations = calibrateHandEye.result()[itmProgress][itmIterations].asInt();
      feedback.residual = getCalibrationResidual(calibrateHandEye.result()[itmProgress]);
      calibrateHandEyeServer->publishFeedback(feedback);

      if (calibrateHandEyeServer->isPreemptRequested())
      {
        NxLibCommand(cmdBreak).execute();

        calibrateHandEyeServer->setPreempted();
        return;
      }

      waitingRate.sleep();
    }

    result.calibration_time =
        calibrateHandEye.result()[itmTime].asDouble() / 1000;  // NxLib time is in milliseconds, ROS
                                                               // expects time to be in seconds.
    result.number_of_iterations = calibrateHandEye.result()[itmIterations].asInt();
    result.residual = getCalibrationResidual(calibrateHandEye.result());

    tf::poseTFToMsg(poseFromNxLib(cameraNode[itmLink]).inverse(), result.link);
    tf::poseTFToMsg(poseFromNxLib(calibrateHandEye.result()[itmPatternPose]), result.pattern_pose);

    if (goal->write_calibration_to_eeprom)
    {
      // Save the new calibration link to the camera's EEPROM.
      NxLibCommand storeCalibration(cmdStoreCalibration);
      storeCalibration.parameters()[itmCameras] = serial;
      storeCalibration.parameters()[itmLink] = true;
      storeCalibration.execute();
    }
  }

  calibrateHandEyeServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(CalibrateHandEye)
}

void Camera::onCalibrateWorkspace(const ensenso_camera_msgs::CalibrateWorkspaceGoalConstPtr& goal)
{
  START_NXLIB_ACTION(CalibrateWorkspace, calibrateWorkspaceServer)

  if (!fixed)
  {
    ROS_WARN("You are performing a workspace calibration for a moving camera. "
             "Are you sure that this is what you want to do?");
  }

  ensenso_camera_msgs::CalibrateWorkspaceResult result;
  result.successful = true;

  loadParameterSet(goal->parameter_set, projectorOff);

  int numberOfShots = goal->number_of_shots;
  if (numberOfShots < 1)
  {
    numberOfShots = 1;
  }

  ros::Time imageTimestamp;
  for (int i = 0; i < numberOfShots; i++)
  {
    PREEMPT_ACTION_IF_REQUESTED

    ros::Time timestamp = capture();
    if (i == 0) imageTimestamp = timestamp;

    PREEMPT_ACTION_IF_REQUESTED

    bool clearBuffer = (i == 0);
    std::vector<CalibrationPattern> patterns = collectPattern(clearBuffer);
    if (patterns.size() != 1)
    {
      result.successful = false;
      calibrateWorkspaceServer->setSucceeded(result);
      return;
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  tf::Transform patternTransformation = estimatePatternPose(imageTimestamp);

  PREEMPT_ACTION_IF_REQUESTED

  tf::Pose definedPatternPose;
  tf::poseMsgToTF(goal->defined_pattern_pose, definedPatternPose);

  NxLibCommand calibrateWorkspace(cmdCalibrateWorkspace);
  calibrateWorkspace.parameters()[itmCameras] = serial;
  writePoseToNxLib(patternTransformation, calibrateWorkspace.parameters()[itmPatternPose]);
  writePoseToNxLib(definedPatternPose, calibrateWorkspace.parameters()[itmDefinedPose]);
  calibrateWorkspace.parameters()[itmTarget] = TARGET_FRAME_LINK;
  calibrateWorkspace.execute();

  if (goal->write_calibration_to_eeprom)
  {
    // Save the new calibration link to the camera's EEPROM.
    NxLibCommand storeCalibration(cmdStoreCalibration);
    storeCalibration.parameters()[itmCameras] = serial;
    storeCalibration.parameters()[itmLink] = true;
    storeCalibration.execute();
  }

  calibrateWorkspaceServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(CalibrateWorkspace)
}

bool Camera::cameraIsAvailable() const
{
  return cameraNode[itmStatus].exists() && cameraNode[itmStatus][itmAvailable].exists() &&
         cameraNode[itmStatus][itmAvailable].asBool();
}

bool Camera::cameraIsOpen() const
{
  return cameraNode[itmStatus].exists() && cameraNode[itmStatus][itmOpen].exists() &&
         cameraNode[itmStatus][itmOpen].asBool();
}

void Camera::publishStatus(ros::TimerEvent const& event) const
{
  std::lock_guard<std::mutex> lock(nxLibMutex);

  diagnostic_msgs::DiagnosticStatus cameraStatus;
  cameraStatus.name = "Camera";
  cameraStatus.hardware_id = serial;
  cameraStatus.level = diagnostic_msgs::DiagnosticStatus::OK;
  cameraStatus.message = "OK";

  if (!cameraIsOpen())
  {
    cameraStatus.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    cameraStatus.message = "Camera is no longer open";
  }

  diagnostic_msgs::DiagnosticArray status;
  status.header.stamp = ros::Time::now();
  status.status.push_back(cameraStatus);
  statusPublisher.publish(status);
}

void Camera::saveDefaultParameterSet()
{
  defaultParameters << cameraNode[itmParameters];
}

void Camera::saveParameterSet(std::string name, bool projectorWasSet)
{
  if (name.empty())
  {
    name = DEFAULT_PARAMETER_SET;
  }

  ParameterSet& parameterSet = parameterSets.at(name);

  parameterSet.node.erase();
  parameterSet.node << cameraNode[itmParameters];

  if (projectorWasSet)
  {
    parameterSet.autoProjector = false;
  }
}

void Camera::loadParameterSet(std::string name, ProjectorState projector)
{
  if (name.empty())
  {
    name = DEFAULT_PARAMETER_SET;
  }

  ROS_DEBUG("Loading parameter set '%s'", name.c_str());

  if (parameterSets.count(name) == 0)
  {
    // The parameter set was never used before. Create it by copying the
    // default settings.
    parameterSets.insert(std::make_pair(name, ParameterSet(name, defaultParameters)));
  }

  ParameterSet const& parameterSet = parameterSets.at(name);

  bool changedParameters = false;
  if (name != currentParameterSet)
  {
    cameraNode[itmParameters] << parameterSet.node;
    changedParameters = true;
  }

  if (parameterSet.autoProjector && projector != projectorDontCare)
  {
    try
    {
      if (projector == projectorOn)
      {
        cameraNode[itmParameters][itmCapture][itmProjector] = true;
        cameraNode[itmParameters][itmCapture][itmFrontLight] = false;
      }
      else
      {
        cameraNode[itmParameters][itmCapture][itmProjector] = false;
        cameraNode[itmParameters][itmCapture][itmFrontLight] = true;
      }
      changedParameters = true;
    }
    catch (...)
    {
      // Setting the projector and front light fails on file cameras.
    }
  }

  if (changedParameters)
  {
    NxLibCommand synchronize(cmdSynchronize);
    synchronize.parameters()[itmCameras] = serial;
    synchronize.execute();
  }

  currentParameterSet = name;
}

ros::Time Camera::capture() const
{
  ROS_DEBUG("Capturing an image...");

  NxLibCommand capture(cmdCapture);
  capture.parameters()[itmCameras] = serial;
  capture.execute();

  NxLibItem imageNode = cameraNode[itmImages][itmRaw];
  if (imageNode.isArray())
  {
    imageNode = imageNode[0];
  }
  imageNode = imageNode[itmLeft];

  return timestampFromNxLibNode(imageNode);
}

std::vector<CalibrationPattern> Camera::collectPattern(bool clearBuffer) const
{
  if (clearBuffer)
  {
    NxLibCommand(cmdDiscardPatterns).execute();
  }

  NxLibCommand collectPattern(cmdCollectPattern);
  collectPattern.parameters()[itmCameras] = serial;
  collectPattern.parameters()[itmDecodeData] = true;
  collectPattern.parameters()[itmFilter][itmCameras] = serial;
  collectPattern.parameters()[itmFilter][itmUseModel] = true;
  collectPattern.parameters()[itmFilter][itmType] = valStatic;
  collectPattern.parameters()[itmFilter][itmValue] = true;
  try
  {
    collectPattern.execute();
  }
  catch (NxLibException& e)
  {
    if (e.getErrorCode() == NxLibExecutionFailed)
    {
      if (collectPattern.result()[itmErrorSymbol] == errPatternNotFound ||
          collectPattern.result()[itmErrorSymbol] == errPatternNotDecodable)
      {
        return {};
      }
    }
    throw;
  }

  if (!collectPattern.result()[itmStereo].exists())
  {
    // We did find patterns, but only in one of the cameras.
    return {};
  }

  std::vector<CalibrationPattern> result;

  for (int i = 0; i < collectPattern.result()[itmStereo].count(); i++)
  {
    result.emplace_back(collectPattern.result()[itmStereo][i]);
  }

  // Extract the pattern's image points from the result.

  NxLibItem pattern = collectPattern.result()[itmPatterns][0][serial];
  if (pattern[itmLeft].count() > 1 || pattern[itmRight].count() > 1)
  {
    // We cannot tell which of the patterns in the two cameras belong together,
    // because that would need a comparison with the stereo model.
    return result;
  }

  for (size_t i = 0; i < result.size(); i++)
  {
    for (int j = 0; j < pattern[itmLeft][i][itmPoints].count(); j++)
    {
      NxLibItem pointNode = pattern[itmLeft][i][itmPoints][j];

      ensenso_camera_msgs::ImagePoint point;
      point.x = pointNode[0].asDouble();
      point.y = pointNode[1].asDouble();
      result[i].leftPoints.push_back(point);
    }
    for (int j = 0; j < pattern[itmRight][i][itmPoints].count(); j++)
    {
      NxLibItem pointNode = pattern[itmRight][i][itmPoints][j];

      ensenso_camera_msgs::ImagePoint point;
      point.x = pointNode[0].asDouble();
      point.y = pointNode[1].asDouble();
      result[i].rightPoints.push_back(point);
    }
  }

  return result;
}

tf::Stamped<tf::Pose> Camera::estimatePatternPose(ros::Time imageTimestamp, std::string const& targetFrame,
                                                  bool latestPatternOnly) const
{
  updateTransformations(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose);
  if (latestPatternOnly)
  {
    estimatePatternPose.parameters()[itmAverage] = false;

    int patternCount = NxLibItem()[itmParameters][itmPatternCount].asInt();
    estimatePatternPose.parameters()[itmFilter][itmOr][0][itmAnd][0][itmType] = valIndex;
    estimatePatternPose.parameters()[itmFilter][itmOr][0][itmAnd][0][itmValue] = patternCount - 1;
  }
  else
  {
    estimatePatternPose.parameters()[itmAverage] = true;
  }
  estimatePatternPose.execute();

  ROS_ASSERT(estimatePatternPose.result()[itmPatterns].count() == 1);

  return poseFromNxLib(estimatePatternPose.result()[itmPatterns][0][itmPatternPose], ros::Time::now(), targetFrame);
}

std::vector<tf::Stamped<tf::Pose>> Camera::estimatePatternPoses(ros::Time imageTimestamp,
                                                                const std::string& targetFrame) const
{
  updateTransformations(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose);
  estimatePatternPose.parameters()[itmAverage] = false;
  estimatePatternPose.parameters()[itmFilter][itmCameras] = serial;
  estimatePatternPose.parameters()[itmFilter][itmUseModel] = true;
  estimatePatternPose.parameters()[itmFilter][itmType] = valStatic;
  estimatePatternPose.parameters()[itmFilter][itmValue] = true;
  estimatePatternPose.execute();

  int numberOfPatterns = estimatePatternPose.result()[itmPatterns].count();

  std::vector<tf::Stamped<tf::Pose>> result;
  result.reserve(numberOfPatterns);

  for (int i = 0; i < numberOfPatterns; i++)
  {
    result.push_back(poseFromNxLib(estimatePatternPose.result()[itmPatterns][i][itmPatternPose],
                                   ros::Time::now(), targetFrame));
  }

  return result;
}

void Camera::updateTransformations(ros::Time time, std::string frame, bool useCachedTransformation) const
{
  if (frame.empty())
  {
    frame = targetFrame;
  }

  // Transformation are represented in the NxLib as follows.
  // The camera's link node contains the calibration data from e.g. the hand
  // eye calibration. This is always used when it is present.
  // The transformation between the camera frame and the target frame (in
  // which the data is returned) is fetched from TF and written to a link node
  // of the NxLib. When the target frame is different from the camera's own
  // frame, we set the camera's link target to this link node.

  if (cameraFrame == frame)
  {
    // The camera frame is the target frame already. No need to transform
    // anything in the NxLib.
    cameraNode[itmLink][itmTarget] = "";
    return;
  }

  cameraNode[itmLink][itmTarget] = TARGET_FRAME_LINK;

  // Update the transformation to the target frame in the NxLib according to
  // the current information from TF.
  tf::StampedTransform transform;
  if (useCachedTransformation && transformationCache.count(frame) != 0)
  {
    transform = transformationCache[frame];
  }
  else
  {
    // TF exceptions will be caught outside and cancel the current action.
    transformListener.waitForTransform(cameraFrame, frame, time, ros::Duration(TRANSFORMATION_REQUEST_TIMEOUT));
    transformListener.lookupTransform(cameraFrame, frame, time, transform);
    transformationCache[frame] = transform;
  }

  writePoseToNxLib(transform, NxLibItem()[itmLinks][TARGET_FRAME_LINK]);
}

void Camera::updateTransformations(tf::Pose const& targetFrameTransformation) const
{
  cameraNode[itmLink][itmTarget] = TARGET_FRAME_LINK;
  writePoseToNxLib(targetFrameTransformation, NxLibItem()[itmLinks][TARGET_FRAME_LINK]);
}

void Camera::fillCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info, bool right, bool rectified) const
{
  info->header.frame_id = cameraFrame;

  info->width = cameraNode[itmSensor][itmSize][0].asInt();
  info->height = cameraNode[itmSensor][itmSize][1].asInt();

  NxLibItem monoCalibrationNode = cameraNode[itmCalibration][itmMonocular][right ? itmRight : itmLeft];
  NxLibItem stereoCalibrationNode = cameraNode[itmCalibration][itmStereo][right ? itmRight : itmLeft];

  if (rectified)
  {
    // For the rectified images all transformations are the identity (because all of the distortions were already
    // removed), except for the stereo camera matrix.

    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info->D.clear();
    info->D.resize(5, 0);

    info->K.fill(0);
    info->P.fill(0);
    info->R.fill(0);
    for (int row = 0; row < 3; row++)
    {
      for (int column = 0; column < 3; column++)
      {
        info->P[4 * row + column] = stereoCalibrationNode[itmCamera][column][row].asDouble();

        if (row == column)
        {
          info->K[3 * row + column] = 1;
          info->R[3 * row + column] = 1;
        }
      }
    }
  }
  else
  {
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info->D.clear();
    for (int i = 0; i < 5; i++)
    {
      info->D.push_back(monoCalibrationNode[itmDistortion][i].asDouble());
    }

    info->K.fill(0);  // The mono camera matrix.
    info->P.fill(0);  // The stereo camera matrix.
    info->R.fill(0);
    for (int row = 0; row < 3; row++)
    {
      for (int column = 0; column < 3; column++)
      {
        info->K[3 * row + column] = monoCalibrationNode[itmCamera][column][row].asDouble();
        info->P[4 * row + column] = stereoCalibrationNode[itmCamera][column][row].asDouble();

        info->R[3 * row + column] = stereoCalibrationNode[itmRotation][column][row].asDouble();
      }
    }
  }

  if (right)
  {
    // Add the offset of the right camera relative to the left one to the projection matrix.
    double fx = stereoCalibrationNode[itmCamera][0][0].asDouble();
    double baseline = cameraNode[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
    info->P[3] = -fx * baseline;
  }

  info->binning_x = cameraNode[itmParameters][itmCapture][itmBinning].asInt();
  info->binning_y = info->binning_x;

  int leftTopX = cameraNode[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0].asInt();
  int leftTopY = cameraNode[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1].asInt();
  int rightBottomX = cameraNode[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0].asInt();
  int rightBottomY = cameraNode[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1].asInt();
  info->roi.x_offset = leftTopX;
  info->roi.y_offset = leftTopY;
  info->roi.width = rightBottomX - leftTopX;
  info->roi.height = rightBottomY - leftTopY;
  if (rectified)
  {
    info->roi.do_rectify = true;
  }
}

void Camera::updateCameraInfo()
{
  fillCameraInfoFromNxLib(leftCameraInfo, false);
  fillCameraInfoFromNxLib(rightCameraInfo, true);
  fillCameraInfoFromNxLib(leftRectifiedCameraInfo, false, true);
  fillCameraInfoFromNxLib(rightRectifiedCameraInfo, true, true);
}

ensenso_camera_msgs::ParameterPtr Camera::readParameter(std::string const& key) const
{
  auto message = boost::make_shared<ensenso_camera_msgs::Parameter>();
  message->key = key;

  if (key == ensenso_camera_msgs::Parameter::REGION_OF_INTEREST)
  {
    PointCloudROI const& roi = parameterSets.at(currentParameterSet).roi;

    message->region_of_interest_value.lower.x = roi.minX;
    message->region_of_interest_value.lower.y = roi.minY;
    message->region_of_interest_value.lower.z = roi.minZ;
    message->region_of_interest_value.upper.x = roi.maxX;
    message->region_of_interest_value.upper.y = roi.maxY;
    message->region_of_interest_value.upper.z = roi.maxZ;
  }
  else if (key == ensenso_camera_msgs::Parameter::FLEX_VIEW)
  {
    NxLibItem flexViewNode = cameraNode[itmParameters][itmCapture][itmFlexView];

    if (!flexViewNode.exists())
    {
      message->float_value = -1;
    }
    else if (flexViewNode.isNumber())
    {
      message->float_value = flexViewNode.asInt();
    }
    else
    {
      message->float_value = 0;
    }
  }
  else if (parameterExists(key))
  {
    // The parameter is mapped to an NxLib node.

    NxLibItem node = parameterNode(cameraNode, key);

    if (node.exists())
    {
      switch (parameterType(key))
      {
        case boolType:
          message->bool_value = node.asBool();
          break;
        case numberType:
          message->float_value = node.asDouble();
          break;
        case stringType:
          message->string_value = node.asString();
          break;
      }
    }
    else
    {
      ROS_WARN("Reading the parameter %s, but the camera does not support it!", key.c_str());
    }
  }
  else
  {
    ROS_ERROR("Unknown parameter '%s'!", key.c_str());
  }

  return message;
}

void Camera::writeParameter(ensenso_camera_msgs::Parameter const& parameter)
{
  if (parameter.key == ensenso_camera_msgs::Parameter::REGION_OF_INTEREST)
  {
    PointCloudROI& roi = parameterSets.at(currentParameterSet).roi;

    roi.minX = parameter.region_of_interest_value.lower.x;
    roi.minY = parameter.region_of_interest_value.lower.y;
    roi.minZ = parameter.region_of_interest_value.lower.z;
    roi.maxX = parameter.region_of_interest_value.upper.x;
    roi.maxY = parameter.region_of_interest_value.upper.y;
    roi.maxZ = parameter.region_of_interest_value.upper.z;

    parameterSets.at(currentParameterSet).useROI = !roi.isEmpty();
  }
  else if (parameter.key == ensenso_camera_msgs::Parameter::FLEX_VIEW)
  {
    NxLibItem flexViewNode = cameraNode[itmParameters][itmCapture][itmFlexView];

    if (!flexViewNode.exists())
    {
      ROS_WARN("Writing the parameter FlexView, but the camera does not support it!");
      return;
    }

    int n = static_cast<int>(std::round(parameter.float_value));
    if (n <= 1)
    {
      flexViewNode = false;
    }
    else
    {
      flexViewNode = n;
    }
  }
  else if (parameterExists(parameter.key))
  {
    // The parameter is mapped to an NxLib node.

    NxLibItem node = parameterNode(cameraNode, parameter.key);

    if (!node.exists())
    {
      ROS_WARN("Writing the parameter %s, but the camera does not support it!", parameter.key.c_str());
      return;
    }

    switch (parameterType(parameter.key))
    {
      case boolType:
        node.set<bool>(parameter.bool_value);
        break;
      case numberType:
        node.set<double>(parameter.float_value);
        break;
      case stringType:
        node.set<std::string>(parameter.string_value);
    }
  }
  else
  {
    ROS_ERROR("Unknown parameter '%s'!", parameter.key.c_str());
  }
}
