#include "ensenso_camera/mono_camera.h"

#include "ensenso_camera/pose_utilities.h"

MonoCamera::MonoCamera(ensenso::ros::NodeHandle& nh, CameraParameters params) : Camera(nh, std::move(params))
{
  cameraInfo = ensenso::std::make_shared<sensor_msgs::msg::CameraInfo>();
  rectifiedCameraInfo = ensenso::std::make_shared<sensor_msgs::msg::CameraInfo>();

  requestDataServer = MAKE_MONO_SERVER(MonoCamera, RequestData, "request_data");
  locatePatternServer = MAKE_MONO_SERVER(MonoCamera, LocatePattern, "locate_pattern");
}

void MonoCamera::advertiseTopics()
{
  rawImagePublisher = ensenso::image_transport::create_camera_publisher(nh, "raw/image");
  rectifiedImagePublisher = ensenso::image_transport::create_camera_publisher(nh, "rectified/image");
}

void MonoCamera::init()
{
  advertiseTopics();
  startServers();
  initTfPublishTimer();
  initStatusTimer();
}

void MonoCamera::startServers() const
{
  Camera::startServers();
  requestDataServer->start();
  locatePatternServer->start();
}

ensenso::ros::Time MonoCamera::capture() const
{
  ENSENSO_DEBUG(nh, "Capturing an image...");

  NxLibCommand capture(cmdCapture, params.serial);
  capture.parameters()[itmCameras] = params.serial;
  if (params.captureTimeout > 0)
  {
    capture.parameters()[itmTimeout] = params.captureTimeout;
  }
  capture.execute();

  NxLibItem imageNode = cameraNode[itmImages][itmRaw];
  if (imageNode.isArray())
  {
    imageNode = imageNode[0];
  }

  if (params.isFileCamera)
  {
    // This workaround is needed, because the timestamp of captures from file cameras will not change over time. When
    // looking up the current tf tree, this will result in errors, because the time of the original timestamp is
    // requested, which lies in the past (and most often longer than the tfBuffer will store the transform!)
    return ensenso::ros::now(nh);
  }

  return timestampFromNxLibNode(imageNode);
}

void MonoCamera::updateCameraInfo()
{
  fillCameraInfoFromNxLib(cameraInfo, false);
  fillCameraInfoFromNxLib(rectifiedCameraInfo, true);
}

void MonoCamera::fillCameraInfoFromNxLib(sensor_msgs::msg::CameraInfoPtr const& info, bool rectified)
{
  Camera::fillBasicCameraInfoFromNxLib(info);

  NxLibItem calibrationNode = cameraNode[itmCalibration];

  if (rectified)
  {
    GET_D_MATRIX(info).resize(5, 0.);
  }
  else
  {
    fillDistortionParamsFromNxLib(calibrationNode[itmDistortion], info);
  }

  for (int row = 0; row < 3; row++)
  {
    for (int column = 0; column < 3; column++)
    {
      GET_K_MATRIX(info)[3 * row + column] = calibrationNode[itmCamera][column][row].asDouble();
      GET_P_MATRIX(info)[4 * row + column] = GET_K_MATRIX(info)[3 * row + column];
      if (row == column)
      {
        GET_R_MATRIX(info)[3 * row + column] = 1.0f;
      }
    }
  }
}

void MonoCamera::onRequestData(ensenso::action::RequestDataMonoGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(RequestDataMono, requestDataServer)

  ensenso::action::RequestDataMonoResult result;
  ensenso::action::RequestDataMonoFeedback feedback;

  bool publishResults = goal->publish_results;
  if (!goal->publish_results && !goal->include_results_in_response)
  {
    publishResults = true;
  }

  bool requestRectified = goal->request_rectified_images;
  if (!goal->request_rectified_images && !goal->request_raw_images)
  {
    requestRectified = true;
  }

  capture();

  PREEMPT_ACTION_IF_REQUESTED

  feedback.images_acquired = true;
  requestDataServer->publishFeedback(feedback);

  if (goal->request_raw_images)
  {
    auto rawImages = imagesFromNxLibNode(cameraNode[itmImages][itmRaw], params.cameraFrame, params.isFileCamera);
    cameraInfo->header.stamp = rawImages[0]->header.stamp;
    cameraInfo->header.frame_id = params.cameraFrame;
    if (goal->include_results_in_response)
    {
      for (auto const& image : rawImages)
      {
        result.raw_images.push_back(*image);
      }
      result.camera_info = *cameraInfo;
    }
    if (publishResults)
    {
      rawImagePublisher.publish(rawImages[0], cameraInfo);
    }
  }

  PREEMPT_ACTION_IF_REQUESTED

  if (requestRectified)
  {
    NxLibCommand rectify(cmdRectifyImages, params.serial);
    rectify.parameters()[itmCameras] = params.serial;
    rectify.execute();

    auto rectifiedImages =
        imagesFromNxLibNode(cameraNode[itmImages][itmRectified], params.cameraFrame, params.isFileCamera);
    rectifiedCameraInfo->header.stamp = rectifiedImages[0]->header.stamp;
    rectifiedCameraInfo->header.frame_id = params.cameraFrame;
    if (goal->include_results_in_response)
    {
      for (auto const& image : rectifiedImages)
      {
        result.rectified_images.push_back(*image);
      }
      result.rectified_camera_info = *rectifiedCameraInfo;
    }
    if (publishResults)
    {
      rectifiedImagePublisher.publish(rectifiedImages[0], rectifiedCameraInfo);
    }
  }

  requestDataServer->setSucceeded(std::move(result));

  FINISH_NXLIB_ACTION(RequestDataMono)
}

void MonoCamera::onSetParameter(ensenso::action::SetParameterGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(SetParameter, setParameterServer)

  ensenso::action::SetParameterResult result;

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

  for (auto const& parameter : goal->parameters)
  {
    writeParameter(parameter);
  }

  // Synchronize to make sure that we read back the correct values.
  NxLibCommand synchronize(cmdSynchronize, params.serial);
  synchronize.parameters()[itmCameras] = params.serial;
  synchronize.execute();

  saveParameterSet(goal->parameter_set);

  // Calibration could have changed after setting new parameters.
  updateCameraInfo();

  // Read back the actual values.
  for (auto const& parameter : goal->parameters)
  {
    result.results.push_back(*readParameter(parameter.key));
  }

  setParameterServer->setSucceeded(std::move(result));

  FINISH_NXLIB_ACTION(SetParameter)
}

void MonoCamera::onLocatePattern(ensenso::action::LocatePatternMonoGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(LocatePatternMono, locatePatternServer)

  ensenso::action::LocatePatternMonoResult result;
  ensenso::action::LocatePatternMonoFeedback feedback;

  loadParameterSet(goal->parameter_set);

  int numberOfShots = goal->number_of_shots;
  if (numberOfShots < 1)
  {
    numberOfShots = 1;
  }

  std::vector<MonoCalibrationPattern> patterns;
  ensenso::ros::Time imageTimestamp;
  for (int i = 0; i < numberOfShots; i++)
  {
    PREEMPT_ACTION_IF_REQUESTED

    ensenso::ros::Time timestamp = capture();
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
  result.mono_patterns.resize(patterns.size());
  for (size_t i = 0; i < patterns.size(); i++)
  {
    patterns[i].writeToMessage(result.mono_patterns[i]);
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

    result.mono_pattern_poses.resize(patternPoses.size());
    for (size_t i = 0; i < patternPoses.size(); i++)
    {
      result.mono_pattern_poses[i] = patternPoses[i];
    }
  }
  else
  {
    // Estimate the pose of a single pattern, averaging over the different shots.
    auto patternPose = estimatePatternPose(imageTimestamp, patternFrame);

    result.mono_pattern_poses.resize(1);
    result.mono_pattern_poses[0] = patternPose;
  }

  if (!goal->tf_frame.empty())
  {
    if (patterns.size() == 1)
    {
      geometry_msgs::msg::TransformStamped transform = transformFromPose(result.mono_pattern_poses[0], goal->tf_frame);
      transformBroadcaster->sendTransform(transform);
    }
    else
    {
      ENSENSO_WARN(nh, "Cannot publish the pattern pose in tf, because there are multiple patterns!");
    }
  }

  locatePatternServer->setSucceeded(std::move(result));

  FINISH_NXLIB_ACTION(LocatePatternMono)
}

std::vector<MonoCalibrationPattern> MonoCamera::collectPattern(bool clearBuffer) const
{
  if (clearBuffer)
  {
    NxLibCommand(cmdDiscardPatterns, params.serial).execute();
  }

  NxLibCommand collectPattern(cmdCollectPattern, params.serial);
  collectPattern.parameters()[itmCameras] = params.serial;
  collectPattern.parameters()[itmDecodeData] = true;
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

  std::vector<MonoCalibrationPattern> result;

  for (int i = 0; i < collectPattern.result()[itmPatterns][0][params.serial].count(); i++)
  {
    result.emplace_back(collectPattern.result()[itmPatterns][0][params.serial][i][itmPattern]);
  }

  // Extract the pattern's image points from the result.
  NxLibItem pattern = collectPattern.result()[itmPatterns][0][params.serial];

  for (size_t i = 0; i < result.size(); i++)
  {
    for (int j = 0; j < pattern[i][itmPoints].count(); j++)
    {
      NxLibItem pointNode = pattern[i][itmPoints][j];

      ensenso::msg::ImagePoint point;
      point.x = pointNode[0].asDouble();
      point.y = pointNode[1].asDouble();
      result[i].points.push_back(point);
    }
  }

  return result;
}

geometry_msgs::msg::PoseStamped MonoCamera::estimatePatternPose(ensenso::ros::Time imageTimestamp,
                                                                std::string const& targetFrame,
                                                                bool latestPatternOnly) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, params.serial);
  estimatePatternPose.parameters()[itmType] = itmMonocular;
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
  ENSENSO_ASSERT(patterns.count() == 1);

  return stampedPoseFromNxLib(patterns[0][itmPatternPose], targetFrame, imageTimestamp);
}

std::vector<geometry_msgs::msg::PoseStamped> MonoCamera::estimatePatternPoses(ensenso::ros::Time imageTimestamp,
                                                                              std::string const& targetFrame) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, params.serial);
  estimatePatternPose.parameters()[itmAverage] = false;
  estimatePatternPose.parameters()[itmType] = itmMonocular;
  estimatePatternPose.parameters()[itmFilter][itmCameras] = params.serial;

  estimatePatternPose.execute();

  auto patterns = estimatePatternPose.result()[itmPatterns];

  std::vector<geometry_msgs::msg::PoseStamped> result;
  result.reserve(patterns.count());

  for (int i = 0; i < patterns.count(); i++)
  {
    result.push_back(stampedPoseFromNxLib(patterns[i][itmPatternPose], targetFrame, imageTimestamp));
  }

  return result;
}
