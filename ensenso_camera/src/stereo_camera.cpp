#include "ensenso_camera/stereo_camera.h"

#include "ensenso_camera/conversion.h"
#include "ensenso_camera/helper.h"
#include "ensenso_camera/image_utilities.h"
#include "ensenso_camera/parameters.h"
#include "ensenso_camera/pose_utilities.h"
#include "ensenso_camera/stereo_camera_helpers.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/distortion_models.h>

#define MAKE_SERVER(TYPE, TAG) ::make_unique<TYPE##Server>(nh, #TAG, boost::bind(&StereoCamera::on##TYPE, this, _1))

StereoCamera::StereoCamera(ros::NodeHandle nh, CameraParameters params) : Camera(nh, std::move(params))
{
  leftCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  rightCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  leftRectifiedCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  rightRectifiedCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();

  fitPrimitiveServer = MAKE_SERVER(FitPrimitive, fit_primitive);
  setParameterServer = MAKE_SERVER(SetParameter, set_parameter);
  requestDataServer = MAKE_SERVER(RequestData, request_data);
  locatePatternServer = MAKE_SERVER(LocatePattern, locate_pattern);
  projectPatternServer = MAKE_SERVER(ProjectPattern, project_pattern);
  calibrateHandEyeServer = MAKE_SERVER(CalibrateHandEye, calibrate_hand_eye);
  calibrateWorkspaceServer = MAKE_SERVER(CalibrateWorkspace, calibrate_workspace);
  telecentricProjectionServer = MAKE_SERVER(TelecentricProjection, project_telecentric);
  texturedPointCloudServer = MAKE_SERVER(TexturedPointCloud, texture_point_cloud);
}

void StereoCamera::updateCameraTypeSpecifics()
{
  if (!hasRightCamera())
  {
    rightCameraInfo = nullptr;
    rightRectifiedCameraInfo = nullptr;
  }
}

void StereoCamera::advertiseTopics()
{
  image_transport::ImageTransport imageTransport(nh);
  leftRawImagePublisher = imageTransport.advertiseCamera("raw/left/image", 1);
  leftRectifiedImagePublisher = imageTransport.advertiseCamera("rectified/left/image", 1);
  if (hasRightCamera())
  {
    rightRawImagePublisher = imageTransport.advertiseCamera("raw/right/image", 1);
    rightRectifiedImagePublisher = imageTransport.advertiseCamera("rectified/right/image", 1);
  }
  if (hasDisparityMap())
  {
    disparityMapPublisher = imageTransport.advertiseCamera("disparity_map", 1);
  }
  depthImagePublisher = imageTransport.advertiseCamera("depth/image", 1);
  projectedImagePublisher = imageTransport.advertise("depth/projected_depth_map", 1);

  pointCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1);
  pointCloudPublisherColor = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("point_cloud_color", 1);
  projectedPointCloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("projected_point_cloud", 1);
}

void StereoCamera::init()
{
  advertiseTopics();
  startServers();
  initTfPublishTimer();
  initStatusTimer();
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

  NxLibCommand fitPrimitives(cmdFitPrimitive, params.serial);
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

  fitPrimitiveServer->setSucceeded(std::move(result));

  FINISH_NXLIB_ACTION(FitPrimitive)
}

void StereoCamera::onRequestData(ensenso_camera_msgs::RequestDataGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(RequestData, requestDataServer)

  ensenso_camera_msgs::RequestDataResult result;
  ensenso_camera_msgs::RequestDataFeedback feedback;

  // Automatically enable publishing if neither publish_reults nor include_results_in_response is enabled.
  bool publishResults = goal->publish_results;
  if (!goal->publish_results && !goal->include_results_in_response)
  {
    publishResults = true;
  }

  // Automatically disable requesting raw images if the camera does not have any.
  bool requestRawImages = goal->request_raw_images;
  if (isXrSeries() && !hasRawImages() && requestRawImages)
  {
    ROS_WARN("XR: Capture mode \"Rectified\", skipping raw images request.");
    requestRawImages = false;
  }

  // Automatically disable requesting rectified images if the camera does not have any.
  bool requestRectifiedImages = goal->request_rectified_images;
  if (isXrSeries() && hasRawImages() && requestRectifiedImages)
  {
    ROS_WARN("XR: Capture mode \"Raw\", skipping rectified images request.");
    requestRectifiedImages = false;
  }

  // Automatically disable requesting raw/rectified images if the camera does not have downloaded any.
  if (isXrSeries() && !hasDownloadedImages() && (requestRawImages || requestRectifiedImages))
  {
    ROS_WARN("XR: Downloading raw/rectified images is disabled, skipping the raw/rectified images request.");
    requestRawImages = false;
    requestRectifiedImages = false;
  }

  bool requestDisparityMap = goal->request_disparity_map;

  bool requestDepthImage = goal->request_depth_image;

  // Automatically request point cloud if no data set is explicitly selected.
  bool requestPointCloud = goal->request_point_cloud || goal->request_depth_image;
  if (!goal->request_raw_images && !goal->request_rectified_images && !goal->request_disparity_map &&
      !goal->request_point_cloud && !goal->request_depth_image && !goal->request_normals)
  {
    requestPointCloud = true;
  }

  bool requestNormals = goal->request_normals;

  // Normals require computePointCloud and computePointCloud requires computeDisparityMap to be called first.
  bool computePointCloud = requestPointCloud || requestNormals;
  bool computeDisparityMap = requestDisparityMap || computePointCloud;

  // Automatically disable requesting the disparity map if the camera does not have one.
  if (!hasDisparityMap())
  {
    requestDisparityMap = false;
  }

  // Automatically disable requesting 3d data if the camera is an XR and only has raw images.
  if (isXrSeries() && hasRawImages() && (computeDisparityMap || computePointCloud))
  {
    ROS_WARN(
        "XR: Capture mode \"Raw\", skipping all 3D data requests! Only raw images are captured and they can only be "
        "used for calibration actions. Rectifying these images afterwards is not possible and they cannot be used to "
        "compute 3D data. If you want to retrieve 3D data, set capture mode to \"Rectified\".");
    requestDisparityMap = false;
    requestDepthImage = false;
    requestPointCloud = false;
    requestNormals = false;
    computePointCloud = false;
    computeDisparityMap = false;
  }

  loadParameterSet(goal->parameter_set, computeDisparityMap ? projectorOn : projectorOff);
  ros::Time imageTimestamp = capture();

  PREEMPT_ACTION_IF_REQUESTED

  feedback.images_acquired = true;
  requestDataServer->publishFeedback(feedback);

  if (requestRawImages)
  {
    auto rawImages = imagePairsFromNxLibNode(cameraNode[itmImages][itmRaw], params.cameraFrame, params.isFileCamera);

    leftCameraInfo->header.stamp = rawImages[0].first->header.stamp;
    if (hasRightCamera())
    {
      rightCameraInfo->header.stamp = leftCameraInfo->header.stamp;
    }

    if (goal->include_results_in_response)
    {
      for (auto const& imagePair : rawImages)
      {
        result.left_raw_images.push_back(*imagePair.first);
        if (hasRightCamera())
        {
          result.right_raw_images.push_back(*imagePair.second);
        }
      }
      result.left_camera_info = *leftCameraInfo;
      if (hasRightCamera())
      {
        result.right_camera_info = *rightCameraInfo;
      }
    }
    if (publishResults)
    {
      // We only publish one of the images on the topic, even if FlexView is enabled.
      leftRawImagePublisher.publish(rawImages[0].first, leftCameraInfo);
      if (hasRightCamera())
      {
        rightRawImagePublisher.publish(rawImages[0].second, rightCameraInfo);
      }
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  // Only call cmdRectifyImages if just the rectified images are requested. Otherwise the rectification is implicitly
  // invoked by cmdComputeDisparityMap, which is more efficient when using CUDA.
  if (requestRectifiedImages && !computeDisparityMap)
  {
    NxLibCommand rectify(cmdRectifyImages, params.serial);
    rectify.parameters()[itmCameras] = params.serial;
    rectify.execute();
  }
  else if (computeDisparityMap)
  {
    // Since the S-series currently uses cmdComputeDisparityMap instead of cmdComputePointMap to get the point map, the
    // global link has to be updated here before cmdComputeDisparityMap is executed.
    updateGlobalLink(imageTimestamp, "", goal->use_cached_transformation);

    NxLibCommand disparityMapCommand(cmdComputeDisparityMap, params.serial);
    disparityMapCommand.parameters()[itmCameras] = params.serial;
    disparityMapCommand.execute();
  }

  if (requestRectifiedImages)
  {
    auto rectifiedImages =
        imagePairsFromNxLibNode(cameraNode[itmImages][itmRectified], params.cameraFrame, params.isFileCamera);

    leftRectifiedCameraInfo->header.stamp = rectifiedImages[0].first->header.stamp;
    if (hasRightCamera())
    {
      rightRectifiedCameraInfo->header.stamp = leftRectifiedCameraInfo->header.stamp;
    }

    if (goal->include_results_in_response)
    {
      for (auto const& imagePair : rectifiedImages)
      {
        result.left_rectified_images.push_back(*imagePair.first);
        if (hasRightCamera())
        {
          result.right_rectified_images.push_back(*imagePair.second);
        }
      }
      result.left_rectified_camera_info = *leftRectifiedCameraInfo;
      if (hasRightCamera())
      {
        result.right_rectified_camera_info = *rightRectifiedCameraInfo;
      }
    }
    if (publishResults)
    {
      // We only publish one of the images on the topic, even if FlexView is enabled.
      leftRectifiedImagePublisher.publish(rectifiedImages[0].first, leftRectifiedCameraInfo);
      if (hasRightCamera())
      {
        rightRectifiedImagePublisher.publish(rectifiedImages[0].second, rightRectifiedCameraInfo);
      }
    }
  }

  auto depthImageCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>(*leftRectifiedCameraInfo);

  addDisparityMapOffset(depthImageCameraInfo);

  if (requestDisparityMap)
  {
    auto disparityMap =
        imageFromNxLibNode(cameraNode[itmImages][itmDisparityMap], params.cameraFrame, params.isFileCamera);

    depthImageCameraInfo->header.stamp = disparityMap->header.stamp;

    if (goal->include_results_in_response)
    {
      result.disparity_map = *disparityMap;
      result.disparity_map_info = *depthImageCameraInfo;
    }
    if (publishResults)
    {
      disparityMapPublisher.publish(disparityMap, depthImageCameraInfo);
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  if (computePointCloud)
  {
    NxLibCommand computePointMap(cmdComputePointMap, params.serial);
    computePointMap.parameters()[itmCameras] = params.serial;
    computePointMap.execute();

    PointCloudROI const* pointCloudROI = 0;
    if (parameterSets.at(currentParameterSet).useROI)
    {
      pointCloudROI = &parameterSets.at(currentParameterSet).roi;
    }

    if (requestPointCloud && !requestNormals)
    {
      auto pointCloud = pointCloudFromNxLib(cameraNode[itmImages][itmPointMap], params.targetFrame, params.isFileCamera,
                                            pointCloudROI);

      if (goal->include_results_in_response)
      {
        pcl::toROSMsg(*pointCloud, result.point_cloud);
      }
      if (publishResults)
      {
        pointCloudPublisher.publish(pointCloud);
      }
    }
    else
    {
      NxLibCommand computeNormals(cmdComputeNormals, params.serial);
      computeNormals.execute();

      auto pointCloud =
          pointCloudWithNormalsFromNxLib(cameraNode[itmImages][itmPointMap], cameraNode[itmImages][itmNormals],
                                         params.targetFrame, params.isFileCamera, pointCloudROI);

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

  if (requestDepthImage)
  {
    // In case camera and target frame are different, the point cloud is recomputed with the relative(toWorld)-flag,
    // such that the resulting point cloud (and thus the depth image) is transformed by the NxLib with the transform
    // stored in the stereo camera's link node.
    if (params.cameraFrame != params.targetFrame)
    {
      NxLibCommand computePointMap(cmdComputePointMap, params.serial);
      computePointMap.parameters()[itmCameras] = params.serial;
      computePointMap.parameters()[itmRelative] = true;
      computePointMap.execute();
    }

    auto depthImage =
        depthImageFromNxLibNode(cameraNode[itmImages][itmPointMap], params.cameraFrame, params.isFileCamera);

    depthImageCameraInfo->header.stamp = depthImage->header.stamp;

    if (goal->include_results_in_response)
    {
      result.depth_image = *depthImage;
      result.depth_image_info = *depthImageCameraInfo;
    }
    if (publishResults)
    {
      depthImagePublisher.publish(depthImage, depthImageCameraInfo);
    }
  }

  requestDataServer->setSucceeded(std::move(result));

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
      server->setAborted(std::move(result));
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
  NxLibCommand synchronize(cmdSynchronize, params.serial);
  synchronize.parameters()[itmCameras] = params.serial;
  synchronize.execute();

  saveParameterSet(goal->parameter_set, projectorChanged);

  // The new parameters might change the camera calibration.
  updateCameraInfo();

  // Read back the actual values.
  for (auto const& parameter : goal->parameters)
  {
    result.results.push_back(*readParameter(parameter.key));
  }

  setParameterServer->setSucceeded(std::move(result));

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
    {
      imageTimestamp = timestamp;
    }

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
      locatePatternServer->setSucceeded(std::move(result));
      return;
    }

    if (patterns.size() > 1)
    {
      // Cannot average multiple shots of multiple patterns. We will cancel the capturing and estimate the pose of each
      // pattern individually.
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

  std::string patternFrame = params.targetFrame;
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
      result.pattern_poses[i] = patternPoses[i];
    }
  }
  else
  {
    // Estimate the pose of a single pattern, averaging over the different shots.
    auto patternPose = estimatePatternPose(imageTimestamp, patternFrame);

    result.pattern_poses.resize(1);
    result.pattern_poses[0] = patternPose;
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
      ROS_WARN("Cannot publish the pattern pose in tf, because there are multiple patterns!");
    }
  }

  // The target frame has changed.
  publishCurrentLinks();
  locatePatternServer->setSucceeded(std::move(result));

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

  tf2::Transform patternPose = fromMsg(goal->pattern_pose);

  NxLibCommand projectPattern(cmdProjectPattern, params.serial);
  projectPattern.parameters()[itmCameras] = params.serial;
  projectPattern.parameters()[itmGridSpacing] = goal->grid_spacing * 1000;
  projectPattern.parameters()[itmGridSize][0] = goal->grid_size_x;
  projectPattern.parameters()[itmGridSize][1] = goal->grid_size_y;
  writeTransformToNxLib(patternPose, projectPattern.parameters()[itmPatternPose]);
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

  projectPatternServer->setSucceeded(std::move(result));

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
    handEyeCalibrationRobotTransforms.clear();
  }
  else if (goal->command == goal->CAPTURE_PATTERN)
  {
    if (params.robotFrame.empty() || params.wristFrame.empty())
    {
      result.error_message = "You need to specify a robot base and wrist frame to do a hand-eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(std::move(result));
      return;
    }

    loadParameterSet(goal->parameter_set, projectorOff);
    ros::Time imageTimestamp = capture();

    PREEMPT_ACTION_IF_REQUESTED

    // Load the pattern buffer that we remembered from the previous calibration steps.
    if (!handEyeCalibrationPatternBuffer.empty())
    {
      NxLibCommand setPatternBuffer(cmdSetPatternBuffer, params.serial);
      setPatternBuffer.parameters()[itmPatterns] << handEyeCalibrationPatternBuffer;
      setPatternBuffer.execute();
    }
    else
    {
      NxLibCommand(cmdDiscardPatterns, params.serial).execute();
    }

    std::vector<StereoCalibrationPattern> patterns = collectPattern();
    if (patterns.empty())
    {
      result.found_pattern = false;
      calibrateHandEyeServer->setSucceeded(std::move(result));
      return;
    }
    if (patterns.size() > 1)
    {
      result.error_message = "Detected multiple calibration patterns during a hand-eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(std::move(result));
      return;
    }

    PREEMPT_ACTION_IF_REQUESTED

    result.found_pattern = true;
    patterns[0].writeToMessage(result.pattern);

    auto patternPose = estimatePatternPose(imageTimestamp, params.cameraFrame, true);
    result.pattern_pose = patternPose.pose;

    PREEMPT_ACTION_IF_REQUESTED

    geometry_msgs::TransformStamped robotPose;
    try
    {
      robotPose = tfBuffer->lookupTransform(params.robotFrame, params.wristFrame, ros::Time(0));
    }
    catch (tf2::TransformException& e)
    {
      result.error_message = std::string("Could not look up the robot pose due to the tf error: ") + e.what();
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(std::move(result));
      return;
    }

    // Remember the newly collected data for the next step.
    NxLibCommand getPatternBuffer(cmdGetPatternBuffer, params.serial);
    getPatternBuffer.execute();
    handEyeCalibrationPatternBuffer = getPatternBuffer.result()[itmPatterns].asJson();

    handEyeCalibrationRobotTransforms.push_back(fromMsg(robotPose));

    result.robot_pose = poseFromTransform(robotPose).pose;
  }
  else if (goal->command == goal->START_CALIBRATION)
  {
    if (handEyeCalibrationRobotTransforms.size() < 5)
    {
      result.error_message = "You need to collect at least 5 patterns before starting a hand-eye calibration!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(std::move(result));
      return;
    }

    // Load the pattern observations.
    size_t numberOfPatterns = 0;
    NxLibCommand setPatternBuffer(cmdSetPatternBuffer, params.serial);
    if (!goal->pattern_observations.empty())
    {
      for (size_t i = 0; i < goal->pattern_observations.size(); i++)
      {
        StereoCalibrationPattern pattern(goal->pattern_observations[i]);

        NxLibItem patternNode = setPatternBuffer.parameters()[itmPatterns][i][params.serial];
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
    auto robotTransforms = handEyeCalibrationRobotTransforms;
    if (!goal->robot_poses.empty())
    {
      robotTransforms.clear();
      for (auto const& pose : goal->robot_poses)
      {
        tf2::Transform transform = fromMsg(pose);
        robotTransforms.push_back(transform);
      }
    }

    if (robotTransforms.size() != numberOfPatterns)
    {
      result.error_message = "The number of pattern observations does not match the number of robot poses!";
      ROS_ERROR("%s", result.error_message.c_str());
      calibrateHandEyeServer->setAborted(std::move(result));
      return;
    }

    // Load the initial guesses from the action goal.
    tf2::Transform linkTransform, patternTransform;
    linkTransform = fromMsg(goal->link);
    patternTransform = fromMsg(goal->pattern_pose);

    NxLibCommand calibrateHandEye(cmdCalibrateHandEye, params.serial);
    calibrateHandEye.parameters()[itmSetup] = params.fixed ? valFixed : valMoving;

    // Check the robot geometry and set the fixed axes accordingly.
    if (goal->robot_geometry == goal->DOF4_FIX_CAMERA_POSE_Z_COMPONENT)
    {
      calibrateHandEye.parameters()[itmFixed][itmLink][itmTranslation][0] = false;
      calibrateHandEye.parameters()[itmFixed][itmLink][itmTranslation][1] = false;
      calibrateHandEye.parameters()[itmFixed][itmLink][itmTranslation][2] = true;
    }
    else if (goal->robot_geometry == goal->DOF4_FIX_PATTERN_POSE_Z_COMPONENT)
    {
      calibrateHandEye.parameters()[itmFixed][itmPatternPose][itmTranslation][0] = false;
      calibrateHandEye.parameters()[itmFixed][itmPatternPose][itmTranslation][1] = false;
      calibrateHandEye.parameters()[itmFixed][itmPatternPose][itmTranslation][2] = true;
    }
    else if (goal->robot_geometry == goal->DOF3_FIX_CAMERA_POSE)
    {
      calibrateHandEye.parameters()[itmFixed][itmLink][itmTranslation][0] = true;
      calibrateHandEye.parameters()[itmFixed][itmLink][itmTranslation][1] = true;
      calibrateHandEye.parameters()[itmFixed][itmLink][itmTranslation][2] = true;
    }
    else if (goal->robot_geometry == goal->DOF3_FIX_PATTERN_POSE)
    {
      calibrateHandEye.parameters()[itmFixed][itmPatternPose][itmTranslation][0] = true;
      calibrateHandEye.parameters()[itmFixed][itmPatternPose][itmTranslation][1] = true;
      calibrateHandEye.parameters()[itmFixed][itmPatternPose][itmTranslation][2] = true;
    }

    // The target node will be reset anyway before we calculate data for the next time.
    calibrateHandEye.parameters()[itmTarget] = getNxLibTargetFrameName();
    if (isValid(linkTransform))
    {
      writeTransformToNxLib(linkTransform.inverse(), calibrateHandEye.parameters()[itmLink]);
    }
    if (isValid(patternTransform))
    {
      writeTransformToNxLib(patternTransform, calibrateHandEye.parameters()[itmPatternPose]);
    }
    for (size_t i = 0; i < robotTransforms.size(); i++)
    {
      writeTransformToNxLib(robotTransforms[i], calibrateHandEye.parameters()[itmTransformations][i]);
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
      if (calibrateHandEye.result()[itmProgress].exists())
      {
        ensenso_camera_msgs::CalibrateHandEyeFeedback feedback;
        feedback.number_of_iterations = calibrateHandEye.result()[itmProgress][itmIterations].asInt();
        feedback.residual = getCalibrationResidual(calibrateHandEye.result()[itmProgress]);
        calibrateHandEyeServer->publishFeedback(feedback);
      }

      if (calibrateHandEyeServer->isPreemptRequested())
      {
        NxLibCommand(cmdBreak, params.serial).execute();

        calibrateHandEyeServer->setPreempted();
        return;
      }

      waitingRate.sleep();
    }

    // NxLib time is in milliseconds, ROS expects time to be in seconds.
    result.calibration_time = calibrateHandEye.result()[itmTime].asDouble() / 1000;

    result.number_of_iterations = calibrateHandEye.result()[itmIterations].asInt();
    result.residual = getCalibrationResidual(calibrateHandEye.result());

    result.link = poseFromTransform(transformFromNxLib(cameraNode[itmLink]).inverse());
    result.pattern_pose = poseFromTransform(transformFromNxLib(calibrateHandEye.result()[itmPatternPose]));

    if (goal->write_calibration_to_eeprom)
    {
      // Save the new calibration link to the camera's EEPROM.
      NxLibCommand storeCalibration(cmdStoreCalibration, params.serial);
      storeCalibration.parameters()[itmCameras] = params.serial;
      storeCalibration.parameters()[itmLink] = true;
      storeCalibration.execute();
    }
  }

  // The target frame has changed.
  publishCurrentLinks();
  calibrateHandEyeServer->setSucceeded(std::move(result));

  FINISH_NXLIB_ACTION(CalibrateHandEye)
}

void StereoCamera::onCalibrateWorkspace(ensenso_camera_msgs::CalibrateWorkspaceGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(CalibrateWorkspace, calibrateWorkspaceServer)

  if (!params.fixed)
  {
    ROS_WARN("You are performing a workspace calibration for a moving camera. Are you sure this is what you want?");
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
      calibrateWorkspaceServer->setSucceeded(std::move(result));
      return;
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  auto patternPose = estimatePatternPose(imageTimestamp);
  tf2::Transform patternTransform = fromMsg(patternPose);

  PREEMPT_ACTION_IF_REQUESTED

  tf2::Transform definedPatternTransform = fromMsg(goal->defined_pattern_pose);

  NxLibCommand calibrateWorkspace(cmdCalibrateWorkspace, params.serial);
  calibrateWorkspace.parameters()[itmCameras] = params.serial;
  writeTransformToNxLib(patternTransform, calibrateWorkspace.parameters()[itmPatternPose]);
  writeTransformToNxLib(definedPatternTransform, calibrateWorkspace.parameters()[itmDefinedPose]);
  calibrateWorkspace.parameters()[itmTarget] = getNxLibTargetFrameName();
  calibrateWorkspace.execute();

  if (goal->write_calibration_to_eeprom)
  {
    // Save the new calibration link to the camera's EEPROM.
    NxLibCommand storeCalibration(cmdStoreCalibration, params.serial);
    storeCalibration.parameters()[itmCameras] = params.serial;
    storeCalibration.parameters()[itmLink] = true;
    storeCalibration.execute();
  }

  publishCurrentLinks();
  calibrateWorkspaceServer->setSucceeded(std::move(result));

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
    result.error.message = "You have to define a valid frame, to which to projection will be published. Aborting";
    ROS_ERROR("%s", result.error.message.c_str());
    telecentricProjectionServer->setAborted(std::move(result));
    return;
  }

  tf2::Transform transform;
  std::string publishingFrame = useViewPose ? params.targetFrame : goal->frame;

  if (useViewPose)
  {
    transform = fromMsg(goal->view_pose);
  }
  else
  {
    transform = getLatestTransform(*tfBuffer, params.targetFrame, goal->frame);
  }

  int pixelScale = goal->pixel_scale != 0. ? goal->pixel_scale : 1;
  double scaling = goal->scaling != 0. ? goal->scaling : 1.0;
  int sizeWidth = goal->size_width != 0 ? goal->size_width : 1024;
  int sizeHeight = goal->size_height != 0. ? goal->size_height : 768;
  bool useOpenGL = goal->use_opengl == 1;
  RenderPointMapParamsTelecentric renderParams(useOpenGL, pixelScale, scaling, sizeWidth, sizeHeight,
                                               std::move(transform));

  NxLibCommand renderPointMap(cmdRenderPointMap, params.serial);

  for (int i = 0; i < static_cast<int>(goal->serials.size()); i++)
  {
    renderPointMap.parameters()[itmCameras][i] = goal->serials[i];
  }
  setRenderParams(renderPointMap.parameters(), &renderParams);

  renderPointMap.execute();

  NxLibItem resultPath = renderPointMap.result()[itmImages][itmRenderPointMap];

  if (!resultPath.exists())
  {
    result.error.message = "Rendered Point Map does not exist in path: " + resultPath.path;
    ROS_ERROR("%s", result.error.message.c_str());
    telecentricProjectionServer->setAborted(std::move(result));
    return;
  }

  if (goal->publish_results || goal->include_results_in_response)
  {
    if (goal->request_point_cloud || (!goal->request_point_cloud && !goal->request_depth_image))
    {
      auto pointCloud = retrieveRenderedPointCloud(renderPointMap.result(), goal->frame, params.isFileCamera);

      if (goal->publish_results)
      {
        projectedPointCloudPublisher.publish(pointCloud);
      }
      if (goal->include_results_in_response)
      {
        pcl::toROSMsg(*pointCloud, result.projected_point_cloud);
        telecentricProjectionServer->setSucceeded(std::move(result));
      }
      else
      {
        telecentricProjectionServer->setSucceeded();
      }
    }

    if (goal->request_depth_image)
    {
      auto renderedImage = retrieveRenderedDepthMap(renderPointMap.result(), goal->frame, params.isFileCamera);

      if (goal->publish_results)
      {
        projectedImagePublisher.publish(renderedImage);
      }
      if (goal->include_results_in_response)
      {
        result.projected_depth_map = *renderedImage;
        telecentricProjectionServer->setSucceeded(std::move(result));
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
    result.error.message = "In Order to use this action, you have to specify one mono serial";
    ROS_ERROR("%s", result.error.message.c_str());
    texturedPointCloudServer->setAborted(std::move(result));
    return;
  }

  double farPlane = goal->far_plane ? goal->far_plane : 10000.;
  double nearPlane = goal->near_plane ? goal->near_plane : -10000.;
  bool useOpenGL = goal->use_opengl == 1;
  bool withTexture = true;

  RenderPointMapParamsProjection renderParams(useOpenGL, farPlane, nearPlane, withTexture);

  NxLibCommand renderPointMap(cmdRenderPointMap, params.serial);
  for (int i = 0; i < static_cast<int>(goal->serials.size()); i++)
  {
    renderPointMap.parameters()[itmCameras][i] = goal->serials[i];
  }
  renderPointMap.parameters()[itmCamera] = goal->mono_serial;
  setRenderParams(renderPointMap.parameters(), &renderParams);

  renderPointMap.execute();

  if (goal->publish_results || goal->include_results_in_response)
  {
    auto cloudColored = retrieveTexturedPointCloud(renderPointMap.result(), params.targetFrame, params.isFileCamera);
    if (goal->publish_results)
    {
      pointCloudPublisherColor.publish(cloudColored);
    }
    if (goal->include_results_in_response)
    {
      pcl::toROSMsg(*cloudColored, result.point_cloud);
      texturedPointCloudServer->setSucceeded(std::move(result));
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
    NxLibCommand synchronize(cmdSynchronize, params.serial);
    synchronize.parameters()[itmCameras] = params.serial;
    synchronize.execute();
  }
}

ros::Time StereoCamera::timestampOfCapturedImage() const
{
  // For file cameras this workaround is needed, because the timestamp of captures from file cameras will not change
  // over time. When looking up the current tf tree, this will result in errors, because the time of the original
  // timestamp is requested, which lies in the past (and most often longer than the tfBuffer will store the transform!).
  if (params.isFileCamera)
  {
    return ros::Time::now();
  }
  // For stereo cameras with only one sensor (S-Series) the image is stored in the raw node.
  else if (isSSeries())
  {
    return timestampFromNxLibNode(cameraNode[itmImages][itmRaw]);
  }
  // For XR cameras the image node depends on the capture mode. In Raw mode it behaves like a normal stereo camera.
  else if (isXrSeries() && !hasRawImages())
  {
    ROS_WARN_ONCE("XR: Using timestamp of first left rectified image in capture mode \"Rectified\".");
    return timestampFromNxLibNode(cameraNode[itmImages][itmRectified][itmLeft]);
  }
  // For stereo cameras with left and right sensor the timestamp of the left sensor is taken as the reference.
  else
  {
    return timestampFromNxLibNode(cameraNode[itmImages][itmRaw][itmLeft]);
  }
}

ros::Time StereoCamera::capture() const
{
  // Update virtual objects. Ignore failures with a simple warning.
  if (params.virtualObjectHandler)
  {
    try
    {
      params.virtualObjectHandler->updateObjectLinks();
    }
    catch (const std::exception& e)
    {
      ROS_WARN("Unable to update virtual objects. Error: %s", e.what());
    }
  }

  ROS_DEBUG("Capturing an image...");

  NxLibCommand capture(cmdCapture, params.serial);
  capture.parameters()[itmCameras] = params.serial;
  if (params.captureTimeout > 0)
  {
    capture.parameters()[itmTimeout] = params.captureTimeout;
  }
  capture.execute();

  return timestampOfCapturedImage();
}

std::vector<StereoCalibrationPattern> StereoCamera::collectPattern(bool clearBuffer) const
{
  if (clearBuffer)
  {
    NxLibCommand(cmdDiscardPatterns, params.serial).execute();
  }

  NxLibCommand collectPattern(cmdCollectPattern, params.serial);
  collectPattern.parameters()[itmCameras] = params.serial;
  collectPattern.parameters()[itmDecodeData] = true;
  collectPattern.parameters()[itmFilter][itmCameras] = params.serial;
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
  NxLibItem pattern = collectPattern.result()[itmPatterns][0][params.serial];
  if (pattern[itmLeft].count() > 1 || pattern[itmRight].count() > 1)
  {
    // We cannot tell which of the patterns in the two cameras belong together, because that would need a comparison
    // with the stereo model.
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

geometry_msgs::PoseStamped StereoCamera::estimatePatternPose(ros::Time imageTimestamp, std::string const& targetFrame,
                                                             bool latestPatternOnly) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, params.serial);
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

  auto patterns = estimatePatternPose.result()[itmPatterns];
  ROS_ASSERT(patterns.count() == 1);

  return stampedPoseFromNxLib(patterns[0][itmPatternPose], targetFrame, imageTimestamp);
}

std::vector<geometry_msgs::PoseStamped> StereoCamera::estimatePatternPoses(ros::Time imageTimestamp,
                                                                           std::string const& targetFrame) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, params.serial);
  estimatePatternPose.parameters()[itmAverage] = false;
  estimatePatternPose.parameters()[itmFilter][itmCameras] = params.serial;
  estimatePatternPose.parameters()[itmFilter][itmUseModel] = true;
  estimatePatternPose.parameters()[itmFilter][itmType] = valStatic;
  estimatePatternPose.parameters()[itmFilter][itmValue] = true;
  estimatePatternPose.execute();

  auto patterns = estimatePatternPose.result()[itmPatterns];

  std::vector<geometry_msgs::PoseStamped> result;
  result.reserve(patterns.count());

  for (int i = 0; i < patterns.count(); i++)
  {
    result.push_back(stampedPoseFromNxLib(patterns[i][itmPatternPose], targetFrame, imageTimestamp));
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
    // For the rectified images all transformations are the identity (because all of the distortions were already
    // removed), except for the stereo camera matrix.
    info->D.resize(5, 0);

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
    fillDistortionParamsFromNxLib(monoCalibrationNode[itmDistortion], info);

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
  fillCameraInfoFromNxLib(leftRectifiedCameraInfo, false, true);
  if (hasRightCamera())
  {
    fillCameraInfoFromNxLib(rightCameraInfo, true);
    fillCameraInfoFromNxLib(rightRectifiedCameraInfo, true, true);
  }
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
  else
  {
    Camera::writeParameter(parameter);
  }
}

bool StereoCamera::isSSeries() const
{
  return cameraNode[itmType].asString() == "StructuredLight";
}

bool StereoCamera::isXrSeries() const
{
  std::string modelName = cameraNode[itmModelName].asString();
  return startswith(modelName, "XR");
}

bool StereoCamera::hasRightCamera() const
{
  return (!isSSeries());
}

bool StereoCamera::hasRawImages() const
{
  std::string captureMode = cameraNode[itmParameters][itmCapture][itmMode].asString();
  return captureMode == valRaw;
}

bool StereoCamera::hasDownloadedImages() const
{
  return isXrSeries() ? cameraNode[itmParameters][itmCapture][itmDownloadImages].asBool() : true;
}

bool StereoCamera::hasDisparityMap() const
{
  return (!isSSeries());
}

void StereoCamera::addDisparityMapOffset(sensor_msgs::CameraInfoPtr const& info) const
{
  double disparityMapOffset = 0.0;
  if (cameraNode[itmCalibration][itmDynamic][itmStereo][itmDisparityMapOffset].exists())
  {
    disparityMapOffset = cameraNode[itmCalibration][itmDynamic][itmStereo][itmDisparityMapOffset].asDouble();
  }

  // Adjust the width, which differs from the image width if it has a non-zero disparity map offset.
  info->width -= disparityMapOffset;

  // Adjust origin cx.
  info->K[2] -= disparityMapOffset;
  info->P[2] -= disparityMapOffset;
}
