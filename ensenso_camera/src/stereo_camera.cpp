#include "ensenso_camera/stereo_camera.h"

#include "ensenso_camera/helper.h"
#include "ensenso_camera/image_utilities.h"
#include "ensenso_camera/parameters.h"
#include "ensenso_camera/pose_utilities.h"
#include "ensenso_camera/nxLibHelpers.h"
#include "ensenso_camera/conversion.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/distortion_models.h>
#include <pcl_ros/point_cloud.h>

StereoCamera::StereoCamera(ros::NodeHandle nh, std::string serial, std::string fileCameraPath, bool fixed,
                           std::string cameraFrame, std::string targetFrame, std::string robotFrame,
                           std::string wristFrame, std::string linkFrame, int captureTimeout)
  : Camera(nh, std::move(serial), std::move(fileCameraPath), fixed, std::move(cameraFrame), std::move(targetFrame),
           std::move(linkFrame))
  , robotFrame(std::move(robotFrame))
  , wristFrame(std::move(wristFrame))
  , captureTimeout(captureTimeout)
{
  leftCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  rightCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  leftRectifiedCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  rightRectifiedCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();

  fitPrimitiveServer =
      ::make_unique<FitPrimitiveServer>(nh, "fit_primitive", boost::bind(&StereoCamera::onFitPrimitive, this, _1));
  setParameterServer =
      ::make_unique<SetParameterServer>(nh, "set_parameter", boost::bind(&StereoCamera::onSetParameter, this, _1));

  requestDataServer =
      ::make_unique<RequestDataServer>(nh, "request_data", boost::bind(&StereoCamera::onRequestData, this, _1));
  locatePatternServer =
      ::make_unique<LocatePatternServer>(nh, "locate_pattern", boost::bind(&StereoCamera::onLocatePattern, this, _1));
  projectPatternServer =
      ::make_unique<ProjectPatternServer>(nh, "project_pattern", boost::bind(&StereoCamera::onProjectPattern, this, _1));
  calibrateHandEyeServer = ::make_unique<CalibrateHandEyeServer>(
      nh, "calibrate_hand_eye", boost::bind(&StereoCamera::onCalibrateHandEye, this, _1));
  calibrateWorkspaceServer = ::make_unique<CalibrateWorkspaceServer>(
      nh, "calibrate_workspace", boost::bind(&StereoCamera::onCalibrateWorkspace, this, _1));
  telecentricProjectionServer = ::make_unique<TelecentricProjectionServer>(
      nh, "project_telecentric", boost::bind(&StereoCamera::onTelecentricProjection, this, _1));
  texturedPointCloudServer = ::make_unique<TexturedPointCloudServer>(
      nh, "texture_point_cloud", boost::bind(&StereoCamera::onTexturedPointCloud, this, _1));

  image_transport::ImageTransport imageTransport(nh);
  leftRawImagePublisher = imageTransport.advertiseCamera("raw/left/image", 1);
  rightRawImagePublisher = imageTransport.advertiseCamera("raw/right/image", 1);
  leftRectifiedImagePublisher = imageTransport.advertiseCamera("rectified/left/image", 1);
  rightRectifiedImagePublisher = imageTransport.advertiseCamera("rectified/right/image", 1);
  disparityMapPublisher = imageTransport.advertise("disparity_map", 1);
  depthImagePublisher = imageTransport.advertiseCamera("depth/image", 1);
  projectedImagePublisher = imageTransport.advertise("depth/projected_depth_map", 1);

  pointCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1);
  pointCloudPublisherColor = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("point_cloud_color", 1);
  projectedPointCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("projected_point_cloud", 1);
}

bool StereoCamera::open()
{
  Camera::open();

  updateCameraInfo();

  return true;
}

void StereoCamera::startServers() const
{
  Camera::startServers();
  requestDataServer->start();
  fitPrimitiveServer->start();
  calibrateHandEyeServer->start();
  calibrateWorkspaceServer->start();
  locatePatternServer->start();
  projectPatternServer->start();
  texturedPointCloudServer->start();
  telecentricProjectionServer->start();
}

void StereoCamera::onFitPrimitive(ensenso_camera_msgs::FitPrimitiveGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(FitPrimitive, fitPrimitiveServer)

  NxLibCommand fitPrimitives(cmdFitPrimitive, serial);
  NxLibItem const& primitives = fitPrimitives.parameters()[itmPrimitive];

  int primitivesCount = 0;
  for (auto const& primitive : goal->primitives)
  {
    if (primitive.type == ensenso_camera_msgs::Primitive::SPHERE)
    {
      primitives[primitivesCount][itmRadius][itmMin] = primitive.min_radius * ensenso_conversion::conversionFactor;
      primitives[primitivesCount][itmRadius][itmMax] = primitive.max_radius * ensenso_conversion::conversionFactor;
      primitives[primitivesCount][itmCount] = primitive.count;
    }
    else if (primitive.type == ensenso_camera_msgs::Primitive::CYLINDER)
    {
      primitives[primitivesCount][itmRadius][itmMin] = primitive.min_radius * ensenso_conversion::conversionFactor;
      primitives[primitivesCount][itmRadius][itmMax] = primitive.min_radius * ensenso_conversion::conversionFactor;
    }
    else if (primitive.type == ensenso_camera_msgs::Primitive::PLANE)
    {
      primitives[primitivesCount][itmCount] = primitive.count;
    }
    primitives[primitivesCount][itmType] = primitive.type;
    primitives[primitivesCount][itmInlierThreshold] = primitive.inlier_threshold * ensenso_conversion::conversionFactor;
    primitives[primitivesCount][itmInlierFraction] = primitive.inlier_fraction_in;

    primitivesCount++;
  }

  NxLibItem const& commandParameters = fitPrimitives.parameters();
  if (goal->normal_radius)
  {
    commandParameters[itmNormal][itmRadius] = goal->normal_radius;
  }

  float const zeroThreshold = 10e-6;
  if (!(std::abs(goal->region_of_interest.lower.x) < zeroThreshold &&
        std::abs(goal->region_of_interest.lower.y) < zeroThreshold &&
        std::abs(goal->region_of_interest.lower.z) < zeroThreshold))
  {
    commandParameters[itmBoundingBox][itmMin] << ensenso_conversion::toEnsensoPoint(goal->region_of_interest.lower);
  }
  if (!(std::abs(goal->region_of_interest.upper.x) < zeroThreshold &&
        std::abs(goal->region_of_interest.upper.y) < zeroThreshold &&
        std::abs(goal->region_of_interest.upper.z) < zeroThreshold))
  {
    commandParameters[itmBoundingBox][itmMax] << ensenso_conversion::toEnsensoPoint(goal->region_of_interest.upper);
  }

  if (goal->failure_probability != 0)
  {
    commandParameters[itmFailureProbability] = goal->failure_probability;
  }

  if (std::abs(goal->inlier_threshold) > zeroThreshold)
  {
    commandParameters[itmInlierThreshold] = goal->inlier_threshold * ensenso_conversion::conversionFactor;
  }
  if (std::abs(goal->inlier_fraction) > zeroThreshold)
  {
    commandParameters[itmInlierFraction] = goal->inlier_fraction;
  }
  if (std::abs(goal->scaling) > zeroThreshold)
  {
    commandParameters[itmScaling] = goal->scaling;
  }
  if (goal->ransac_iterations != 0)
  {
    commandParameters[itmIterations] = goal->ransac_iterations;
  }

  fitPrimitives.execute();

  ensenso_camera_msgs::FitPrimitiveResult result;

  NxLibItem const primitiveResults = fitPrimitives.result()[itmPrimitive];
  if (primitiveResults.isArray())
  {
    result.primitives.reserve(primitiveResults.count());
    for (int primitiveCount = 0; primitiveCount < primitiveResults.count(); primitiveCount++)
    {
      NxLibItem const& currentPrimitive = primitiveResults[primitiveCount];
      ensenso_camera_msgs::Primitive primitive;

      primitive.type = currentPrimitive[itmType].asString();
      primitive.ransac_iterations = currentPrimitive[itmIterations].asInt();
      primitive.inlier_count = currentPrimitive[itmInlierCount].asInt();
      primitive.inlier_fraction_out = currentPrimitive[itmInlierFraction].asDouble();
      primitive.score = currentPrimitive[itmScore].asDouble();
      primitive.center = ensenso_conversion::toRosPoint(currentPrimitive[itmCenter]);

      if (primitive.type == ensenso_camera_msgs::Primitive::PLANE)
      {
        primitive.axes.push_back(ensenso_conversion::toRosPoint(currentPrimitive[itmAxis][0]));
        primitive.axes.push_back(ensenso_conversion::toRosPoint(currentPrimitive[itmAxis][1]));
        primitive.normal = ensenso_conversion::toRosPoint(currentPrimitive[itmNormal], false);
      }
      else if (primitive.type == ensenso_camera_msgs::Primitive::SPHERE)
      {
        primitive.radius = currentPrimitive[itmRadius].asDouble() / ensenso_conversion::conversionFactor;
      }
      else if (primitive.type == ensenso_camera_msgs::Primitive::CYLINDER)
      {
        primitive.axis = ensenso_conversion::toRosPoint(currentPrimitive[itmAxis]);
      }
      result.primitives.emplace_back(primitive);
    }
  }

  fitPrimitiveServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(FitPrimitive)
}

void StereoCamera::onRequestData(ensenso_camera_msgs::RequestDataGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(RequestData, requestDataServer)

  ensenso_camera_msgs::RequestDataResult result;
  ensenso_camera_msgs::RequestDataFeedback feedback;

  bool publishResults = goal->publish_results;
  if (!goal->publish_results && !goal->include_results_in_response)
  {
    publishResults = true;
  }

  bool requestPointCloud = goal->request_point_cloud || goal->request_depth_image;
  if (!goal->request_raw_images && !goal->request_rectified_images && !goal->request_point_cloud &&
      !goal->request_normals && !goal->request_depth_image)
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
    auto rawImages = imagePairsFromNxLibNode(cameraNode[itmImages][itmRaw], cameraFrame);

    leftCameraInfo->header.stamp = rawImages[0].first->header.stamp;
    leftCameraInfo->header.frame_id = cameraFrame;
    rightCameraInfo->header.stamp = leftCameraInfo->header.stamp;
    rightCameraInfo->header.frame_id = cameraFrame;

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
      // We only publish one of the images on the topic, even if FlexView is
      // enabled.
      leftRawImagePublisher.publish(rawImages[0].first, leftCameraInfo);
      rightRawImagePublisher.publish(rawImages[0].second, rightCameraInfo);
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  // If we need the disparity map, we do the rectification implicitly in
  // cmdComputeDisparityMap. This is more efficient when using CUDA.
  if (goal->request_rectified_images && !computeDisparityMap)
  {
    NxLibCommand rectify(cmdRectifyImages, serial);
    rectify.parameters()[itmCameras] = serial;
    rectify.execute();
  }
  else if (computeDisparityMap)
  {
    NxLibCommand disparityMapCommand(cmdComputeDisparityMap, serial);
    disparityMapCommand.parameters()[itmCameras] = serial;
    disparityMapCommand.execute();
  }

  if (goal->request_rectified_images)
  {
    auto rectifiedImages = imagePairsFromNxLibNode(cameraNode[itmImages][itmRectified], cameraFrame);

    leftRectifiedCameraInfo->header.stamp = rectifiedImages[0].first->header.stamp;
    leftRectifiedCameraInfo->header.frame_id = cameraFrame;
    rightRectifiedCameraInfo->header.stamp = leftRectifiedCameraInfo->header.stamp;
    rightRectifiedCameraInfo->header.frame_id = cameraFrame;

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
      // We only publish one of the images on the topic, even if FlexView is
      // enabled.
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
    updateGlobalLink(imageTimestamp, "", goal->use_cached_transformation);

    NxLibCommand computePointMap(cmdComputePointMap, serial);
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

    if (goal->request_depth_image)
    {
      if (cameraFrame == targetFrame)
      {
        auto depthImage = depthImageFromNxLibNode(cameraNode[itmImages][itmPointMap], targetFrame);

        if (goal->include_results_in_response)
        {
          result.depth_image = *depthImage;
        }

        if (publishResults)
        {
          depthImagePublisher.publish(depthImage, leftRectifiedCameraInfo);
        }
      }
      else
      {
        ROS_WARN_ONCE("Currently it is not possible to determine the depth map once the stereo camera is extrinsically "
                      "calibrated because the point cloud is then transformed internally. If you want to have a depth "
                      "map, the camera must not be extrinsically calibrated (workspace-/ hand-eye calibration), or "
                      "have a link to another camera.");
      }
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  if (goal->request_normals)
  {
    NxLibCommand computeNormals(cmdComputeNormals, serial);
    computeNormals.execute();

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

void StereoCamera::onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal)
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
  NxLibCommand synchronize(cmdSynchronize, serial);
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

void StereoCamera::onLocatePattern(ensenso_camera_msgs::LocatePatternGoalConstPtr const& goal)
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

  std::vector<StereoCalibrationPattern> patterns;
  ros::Time imageTimestamp;
  for (int i = 0; i < numberOfShots; i++)
  {
    PREEMPT_ACTION_IF_REQUESTED

    ros::Time timestamp = capture();
    if (i == 0)
      imageTimestamp = timestamp;

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
    patterns[i].writeToMessage(result.patterns[i]);
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
      result.pattern_poses[i] = stampedPoseFromTransform(patternPoses[i]);
    }
  }
  else
  {
    // Estimate the pose of a single pattern, averaging over the different
    // shots.
    auto patternPose = estimatePatternPose(imageTimestamp, patternFrame);

    result.pattern_poses.resize(1);
    result.pattern_poses[0] = stampedPoseFromTransform(patternPose);
  }

  if (!goal->tf_frame.empty())
  {
    if (patterns.size() == 1)
    {
      auto transform = transformFromPose(result.pattern_poses[0], goal->tf_frame);
      transformBroadcaster->sendTransform(transform);
    }
    else
    {
      ROS_WARN("Cannot publish the pattern pose in TF, because there are "
               "multiple patterns!");
    }
  }

  // The target frame has changed.
  publishCurrentLinks();
  locatePatternServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(LocatePattern)
}

void StereoCamera::onProjectPattern(ensenso_camera_msgs::ProjectPatternGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(ProjectPattern, projectPatternServer)

  ensenso_camera_msgs::ProjectPatternResult result;

  tf2::Transform targetFrameTransformation = fromMsg(goal->target_frame_transformation);
  if (isValid(targetFrameTransformation))
  {
    updateTransformations(targetFrameTransformation);
  }
  else
  {
    updateGlobalLink();
  }

  PREEMPT_ACTION_IF_REQUESTED

  if (!checkNxLibVersion(2, 1))
  {
    // In old NxLib versions, the project pattern command does not take the grid
    // spacing as a parameter.
    NxLibItem()[itmParameters][itmPattern][itmGridSpacing] = goal->grid_spacing * 1000;
  }

  tf2::Transform patternPose = fromMsg(goal->pattern_pose);

  NxLibCommand projectPattern(cmdProjectPattern, serial);
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

void StereoCamera::onCalibrateHandEye(ensenso_camera_msgs::CalibrateHandEyeGoalConstPtr const& goal)
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
      NxLibCommand setPatternBuffer(cmdSetPatternBuffer, serial);
      setPatternBuffer.parameters()[itmPatterns] << handEyeCalibrationPatternBuffer;
      setPatternBuffer.execute();
    }
    else
    {
      NxLibCommand(cmdDiscardPatterns, serial).execute();
    }

    std::vector<StereoCalibrationPattern> patterns = collectPattern();
    if (patterns.empty())
    {
      result.found_pattern = false;
      calibrateHandEyeServer->setSucceeded(result);
      return;
    }
    if (patterns.size() > 1)
    {
      result.error_message = "Detected multiple calibration patterns during a "
                             "hand eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    PREEMPT_ACTION_IF_REQUESTED

    result.found_pattern = true;
    patterns[0].writeToMessage(result.pattern);

    auto patternPose = estimatePatternPose(imageTimestamp, cameraFrame, true);
    result.pattern_pose = stampedPoseFromTransform(patternPose).pose;

    PREEMPT_ACTION_IF_REQUESTED

    geometry_msgs::TransformStamped robotPose;
    try
    {
      robotPose = tfBuffer.lookupTransform(robotFrame, wristFrame, ros::Time(0));
    }
    catch (tf2::TransformException& e)
    {
      result.error_message = std::string("Could not look up the robot pose due to the TF error: ") + e.what();
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    // Remember the newly collected data for the next step.
    NxLibCommand getPatternBuffer(cmdGetPatternBuffer, serial);
    getPatternBuffer.execute();
    handEyeCalibrationPatternBuffer = getPatternBuffer.result()[itmPatterns].asJson();

    handEyeCalibrationRobotPoses.push_back(fromStampedMessage(robotPose));

    result.robot_pose = stampedPoseFromTransform(robotPose).pose;
  }
  else if (goal->command == goal->START_CALIBRATION)
  {
    if (handEyeCalibrationRobotPoses.size() < 5)
    {
      result.error_message = "You need collect at least 5 patterns before "
                             "starting a hand eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    // Load the pattern observations.
    size_t numberOfPatterns = 0;
    NxLibCommand setPatternBuffer(cmdSetPatternBuffer, serial);
    if (!goal->pattern_observations.empty())
    {
      for (size_t i = 0; i < goal->pattern_observations.size(); i++)
      {
        StereoCalibrationPattern pattern(goal->pattern_observations[i]);

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
    if (!goal->robot_poses.empty())
    {
      robotPoses.clear();
      for (auto const& pose : goal->robot_poses)
      {
        tf2::Transform tfPose = fromMsg(pose);
        robotPoses.push_back(tfPose);
      }
    }

    if (robotPoses.size() != numberOfPatterns)
    {
      result.error_message = "The number of pattern observations does not "
                             "match the number of robot poses!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(result);
      return;
    }

    // Load the initial guesses from the action goal.
    tf2::Transform link, patternPose;
    link = fromMsg(goal->link);
    patternPose = fromMsg(goal->pattern_pose);

    NxLibCommand calibrateHandEye(cmdCalibrateHandEye, serial);
    calibrateHandEye.parameters()[itmSetup] = fixed ? valFixed : valMoving;
    // The target node will be reset anyway before we calculate data for the next time.
    calibrateHandEye.parameters()[itmTarget] = TARGET_FRAME_LINK + "_" + serial;
    if (isValid(link))
    {
      writePoseToNxLib(link.inverse(), calibrateHandEye.parameters()[itmLink]);
    }
    if (isValid(patternPose))
    {
      writePoseToNxLib(patternPose, calibrateHandEye.parameters()[itmPatternPose]);
    }
    for (size_t i = 0; i < robotPoses.size(); i++)
    {
      writePoseToNxLib(robotPoses[i], calibrateHandEye.parameters()[itmTransformations][i]);
    }

    calibrateHandEye.execute(false);

    auto getCalibrationResidual = [](NxLibItem const& node) {  // NOLINT
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
        NxLibCommand(cmdBreak, serial).execute();

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

    result.link = poseFromTransform(poseFromNxLib(cameraNode[itmLink]).inverse());
    result.pattern_pose = poseFromTransform(poseFromNxLib(calibrateHandEye.result()[itmPatternPose]));

    if (goal->write_calibration_to_eeprom)
    {
      // Save the new calibration link to the camera's EEPROM.
      NxLibCommand storeCalibration(cmdStoreCalibration, serial);
      storeCalibration.parameters()[itmCameras] = serial;
      storeCalibration.parameters()[itmLink] = true;
      storeCalibration.execute();
    }
  }

  // The target frame has changed.
  publishCurrentLinks();
  calibrateHandEyeServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(CalibrateHandEye)
}

void StereoCamera::onCalibrateWorkspace(const ensenso_camera_msgs::CalibrateWorkspaceGoalConstPtr& goal)
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
    if (i == 0)
      imageTimestamp = timestamp;

    PREEMPT_ACTION_IF_REQUESTED

    bool clearBuffer = (i == 0);
    std::vector<StereoCalibrationPattern> patterns = collectPattern(clearBuffer);
    if (patterns.size() != 1)
    {
      result.successful = false;
      calibrateWorkspaceServer->setSucceeded(result);
      return;
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  tf2::Transform patternTransformation = fromStampedMessage(estimatePatternPose(imageTimestamp));

  PREEMPT_ACTION_IF_REQUESTED

  tf2::Transform definedPatternPose = fromMsg(goal->defined_pattern_pose);

  NxLibCommand calibrateWorkspace(cmdCalibrateWorkspace, serial);
  calibrateWorkspace.parameters()[itmCameras] = serial;
  writePoseToNxLib(patternTransformation, calibrateWorkspace.parameters()[itmPatternPose]);
  writePoseToNxLib(definedPatternPose, calibrateWorkspace.parameters()[itmDefinedPose]);
  calibrateWorkspace.parameters()[itmTarget] = TARGET_FRAME_LINK + "_" + serial;
  calibrateWorkspace.execute();

  if (goal->write_calibration_to_eeprom)
  {
    // Save the new calibration link to the camera's EEPROM.
    NxLibCommand storeCalibration(cmdStoreCalibration, serial);
    storeCalibration.parameters()[itmCameras] = serial;
    storeCalibration.parameters()[itmLink] = true;
    storeCalibration.execute();
  }

  publishCurrentLinks();
  calibrateWorkspaceServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(CalibrateWorkspace)
}

void StereoCamera::onTelecentricProjection(ensenso_camera_msgs::TelecentricProjectionGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(TelecentricProjection, telecentricProjectionServer)

  ensenso_camera_msgs::TelecentricProjectionResult result;

  bool useViewPose = isValid(goal->view_pose);
  bool frameGiven = !goal->frame.empty();

  if (!frameGiven)
  {
    std::string error = "You have to define a valid frame, to which to projection will be published. Aborting";
    ROS_ERROR("%s", error.c_str());
    result.error.message = error;
    telecentricProjectionServer->setAborted(result);
    return;
  }

  tf2::Transform transform;
  std::string publishingFrame = useViewPose ? targetFrame : goal->frame;

  if (useViewPose)
  {
    transform = fromMsg(goal->view_pose);
  }
  else
  {
    transform = getLatestTransform(tfBuffer, targetFrame, goal->frame);
  }

  int pixelScale = goal->pixel_scale != 0. ? goal->pixel_scale : 1;
  double scaling = goal->scaling != 0. ? goal->scaling : 1.0;
  int sizeWidth = goal->size_width != 0 ? goal->size_width : 1024;
  int sizeHeight = goal->size_height != 0. ? goal->size_height : 768;
  bool useOpenGL = goal->use_opengl == 1;
  RenderPointMapParamsTelecentric params(useOpenGL, pixelScale, scaling, sizeWidth, sizeHeight, transform);

  NxLibCommand renderPointMap(cmdRenderPointMap, serial);

  for (int i = 0; i < static_cast<int>(goal->serials.size()); i++)
  {
    renderPointMap.parameters()[itmCameras][i] = goal->serials[i];
  }
  setRenderParams(renderPointMap.parameters(), nxLibVersion, &params);

  renderPointMap.execute();

  NxLibItem resultPath = retrieveResultPath(renderPointMap.result(), nxLibVersion);

  if (!resultPath.exists())
  {
    std::string error = "Rendered Point Map does not exist in path: " + resultPath.path;
    result.error.message = error;
    ROS_ERROR("%s", error.c_str());
    telecentricProjectionServer->setAborted(result);
    return;
  }

  if (goal->publish_results || goal->include_results_in_response)
  {
    if (goal->request_point_cloud || (!goal->request_point_cloud && !goal->request_depth_image))
    {
      auto pointCloud = retrieveRenderedPointCloud(renderPointMap.result(), nxLibVersion, goal->frame);

      if (goal->publish_results)
      {
        projectedPointCloudPublisher.publish(*pointCloud);
      }
      if (goal->include_results_in_response)
      {
        pcl::toROSMsg(*pointCloud, result.projected_point_cloud);
        telecentricProjectionServer->setSucceeded(result);
      }
      else
      {
        telecentricProjectionServer->setSucceeded();
      }
    }

    if (goal->request_depth_image)
    {
      auto renderedImage = retrieveRenderedDepthMap(renderPointMap.result(), nxLibVersion, goal->frame);

      if (goal->publish_results)
      {
        projectedImagePublisher.publish(renderedImage);
      }
      if (goal->include_results_in_response)
      {
        result.projected_depth_map = *renderedImage;
        telecentricProjectionServer->setSucceeded(result);
      }
      else
      {
        telecentricProjectionServer->setSucceeded();
      }
    }
  }
  else
  {
    telecentricProjectionServer->setSucceeded();
  }

  FINISH_NXLIB_ACTION(TelecentricProjection)
}

void StereoCamera::onTexturedPointCloud(ensenso_camera_msgs::TexturedPointCloudGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(TexturedPointCloud, texturedPointCloudServer)

  ensenso_camera_msgs::TexturedPointCloudResult result;

  if (goal->mono_serial.empty())
  {
    std::string error = "In Order to use this action, you have to specify one mono serial";
    ROS_ERROR("%s", error.c_str());
    result.error.message = error;
    texturedPointCloudServer->setAborted(result);
    return;
  }

  double farPlane = goal->far_plane ? goal->far_plane : 10000.;
  double nearPlane = goal->near_plane ? goal->near_plane : -10000.;
  bool useOpenGL = goal->use_opengl == 1;
  bool withTexture = true;

  RenderPointMapParamsProjection params(useOpenGL, farPlane, nearPlane, withTexture);

  NxLibCommand renderPointMap(cmdRenderPointMap, serial);
  for (int i = 0; i < static_cast<int>(goal->serials.size()); i++)
  {
    renderPointMap.parameters()[itmCameras][i] = goal->serials[i];
  }
  renderPointMap.parameters()[itmCamera] = goal->mono_serial;
  setRenderParams(renderPointMap.parameters(), nxLibVersion, &params);

  renderPointMap.execute();

  if (goal->publish_results || goal->include_results_in_response)
  {
    auto cloudColored = retrieveTexturedPointCloud(renderPointMap.result(), nxLibVersion, targetFrame);
    if (goal->publish_results)
    {
      pointCloudPublisherColor.publish(cloudColored);
    }
    if (goal->include_results_in_response)
    {
      pcl::toROSMsg(*cloudColored, result.point_cloud);
      texturedPointCloudServer->setSucceeded(result);
    }
    else
    {
      texturedPointCloudServer->setSucceeded();
    }
  }
  else
  {
    texturedPointCloudServer->setSucceeded();
  }

  FINISH_NXLIB_ACTION(TexturedPointCloud)
}

void StereoCamera::saveParameterSet(std::string name, bool projectorWasSet)
{
  if (name.empty())
  {
    name = DEFAULT_PARAMETER_SET;
  }

  Camera::saveParameterSet(name);

  ParameterSet& parameterSet = parameterSets.at(name);
  if (projectorWasSet)
  {
    parameterSet.autoProjector = false;
  }
}

void StereoCamera::loadParameterSet(std::string name, ProjectorState projector)
{
  Camera::loadParameterSet(std::move(name));

  bool changedParameters = false;
  ParameterSet const& parameterSet = parameterSets.at(currentParameterSet);

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
    NxLibCommand synchronize(cmdSynchronize, serial);
    synchronize.parameters()[itmCameras] = serial;
    synchronize.execute();
  }
}

ros::Time StereoCamera::capture() const
{
  ROS_DEBUG("Capturing an image...");

  NxLibCommand capture(cmdCapture, serial);
  capture.parameters()[itmCameras] = serial;
  if (captureTimeout > 0)
  {
    capture.parameters()[itmTimeout] = captureTimeout;
  }
  capture.execute();

  NxLibItem imageNode = cameraNode[itmImages][itmRaw];
  imageNode = imageNode[itmLeft];

  if (isFileCamera)
  {
    // This workaround is needed, because the timestamp of captures from file cameras will not change over time. When
    // looking up the current tf tree, this will result in errors, because the time of the original timestamp is
    // requested, which lies in the past (and most often longer than the tfBuffer will store the transform!)
    return ros::Time::now();
  }

  return timestampFromNxLibNode(imageNode);
}

std::vector<StereoCalibrationPattern> StereoCamera::collectPattern(bool clearBuffer) const
{
  if (clearBuffer)
  {
    NxLibCommand(cmdDiscardPatterns, serial).execute();
  }

  NxLibCommand collectPattern(cmdCollectPattern, serial);
  collectPattern.parameters()[itmCameras] = serial;
  collectPattern.parameters()[itmDecodeData] = true;
  collectPattern.parameters()[itmFilter][itmCameras] = serial;
  bool useModel = true;
  collectPattern.parameters()[itmFilter][itmUseModel] = useModel;
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

  std::vector<StereoCalibrationPattern> result;

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
      result.at(i).leftPoints.push_back(point);
    }
    for (int j = 0; j < pattern[itmRight][i][itmPoints].count(); j++)
    {
      NxLibItem pointNode = pattern[itmRight][i][itmPoints][j];

      ensenso_camera_msgs::ImagePoint point;
      point.x = pointNode[0].asDouble();
      point.y = pointNode[1].asDouble();
      result.at(i).rightPoints.push_back(point);
    }
  }

  return result;
}

void StereoCamera::fillCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info, bool right, bool rectified) const
{
  Camera::fillBasicCameraInfoFromNxLib(info);

  NxLibItem monoCalibrationNode = cameraNode[itmCalibration][itmMonocular][right ? itmRight : itmLeft];
  NxLibItem stereoCalibrationNode = cameraNode[itmCalibration][itmStereo][right ? itmRight : itmLeft];

  if (rectified)
  {
    // For the rectified images all transformations are the identity (because
    // all of the distortions were already removed), except for the stereo
    // camera matrix.

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
    fillDistortionParamsFromNxLib(monoCalibrationNode[itmDistortion], info);

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
    // Add the offset of the right camera relative to the left one to the
    // projection matrix.
    double fx = stereoCalibrationNode[itmCamera][0][0].asDouble();
    double baseline = cameraNode[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
    info->P[3] = -fx * baseline;
  }

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

void StereoCamera::updateCameraInfo()
{
  fillCameraInfoFromNxLib(leftCameraInfo, false);
  fillCameraInfoFromNxLib(rightCameraInfo, true);
  fillCameraInfoFromNxLib(leftRectifiedCameraInfo, false, true);
  fillCameraInfoFromNxLib(rightRectifiedCameraInfo, true, true);
}

ensenso_camera_msgs::ParameterPtr StereoCamera::readParameter(std::string const& key) const
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
  else
  {
    message = Camera::readParameter(key);
  }

  return message;
}

void StereoCamera::writeParameter(ensenso_camera_msgs::Parameter const& parameter)
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
      ROS_WARN("Writing the parameter FlexView, but the camera does not "
               "support it!");
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
  else
  {
    Camera::writeParameter(parameter);
  }
}
