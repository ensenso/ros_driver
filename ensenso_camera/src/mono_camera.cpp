#include "ensenso_camera/mono_camera.h"

#include "ensenso_camera/pose_utilities.h"
#include <sensor_msgs/distortion_models.h>

MonoCamera::MonoCamera(ros::NodeHandle nh, std::string serial, std::string fileCameraPath, bool fixed,
                       std::string cameraFrame, std::string targetFrame, std::string linkFrame)
  : Camera(nh, std::move(serial), std::move(fileCameraPath), fixed, std::move(cameraFrame), std::move(targetFrame),
           std::move(linkFrame))
{
  cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
  rectifiedCameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();

  requestDataServer =
      ::make_unique<RequestDataMonoServer>(nh, "request_data", boost::bind(&MonoCamera::onRequestData, this, _1));
  locatePatternServer =
      ::make_unique<LocatePatternMonoServer>(nh, "locate_pattern", boost::bind(&MonoCamera::onLocatePattern, this, _1));

  image_transport::ImageTransport imageTransport(nh);
  rawImagePublisher = imageTransport.advertiseCamera("raw/image", 1);
  rectifiedImagePublisher = imageTransport.advertiseCamera("rectified/image", 1);
}

ros::Time MonoCamera::capture() const
{
  ROS_DEBUG("Capturing an image...");

  NxLibCommand capture(cmdCapture, serial);
  capture.parameters()[itmCameras] = serial;
  capture.execute();

  NxLibItem imageNode = cameraNode[itmImages][itmRaw];
  if (imageNode.isArray())
  {
    imageNode = imageNode[0];
  }

  if (isFileCamera)
  {
    // This workaround is needed, because the timestamp of captures from file cameras will not change over time. When
    // looking up the current tf tree, this will result in errors, because the time of the original timestamp is
    // requested, which lies in the past (and most often longer than the tfBuffer will store the transform!)
    return ros::Time::now();
  }

  return timestampFromNxLibNode(imageNode);
}

bool MonoCamera::open()
{
  Camera::open();

  updateCameraInfo();

  return true;
}

void MonoCamera::updateCameraInfo()
{
  fillCameraInfoFromNxLib(cameraInfo, false);
  fillCameraInfoFromNxLib(rectifiedCameraInfo, true);
}

void MonoCamera::fillCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info, bool rectified)
{
  Camera::fillBasicCameraInfoFromNxLib(info);

  NxLibItem calibrationNode = cameraNode[itmCalibration];

  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info->D.clear();

  if(rectified) {
    info->D.resize(5,0.);
  } else {
    fillDistortionParamsFromNxLib(calibrationNode[itmDistortion], info);
  }

  info->K.fill(0);
  info->P.fill(0);
  info->R.fill(0);
  for (int row = 0; row < 3; row++)
  {
    for (int column = 0; column < 3; column++)
    {
      info->K[3 * row + column] = calibrationNode[itmCamera][column][row].asDouble();
      info->P[4 * row + column] = info->K[3 * row + column];
      if (row == column)
      {
        info->R[3 * row + column] = 1.0f;
      }
    }
  }
}

void MonoCamera::startServers() const
{
  Camera::startServers();
  requestDataServer->start();
  locatePatternServer->start();
}

void MonoCamera::onRequestData(ensenso_camera_msgs::RequestDataMonoGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(RequestDataMono, requestDataServer)

  ensenso_camera_msgs::RequestDataMonoResult result;
  ensenso_camera_msgs::RequestDataMonoFeedback feedback;

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
    auto rawImages = imagesFromNxLibNode(cameraNode[itmImages][itmRaw], cameraFrame);
    cameraInfo->header.stamp = rawImages[0]->header.stamp;
    cameraInfo->header.frame_id = cameraFrame;
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
    NxLibCommand rectify(cmdRectifyImages, serial);
    rectify.parameters()[itmCameras] = serial;
    rectify.execute();

    auto rectifiedImages = imagesFromNxLibNode(cameraNode[itmImages][itmRectified], cameraFrame);
    rectifiedCameraInfo->header.stamp = rectifiedImages[0]->header.stamp;
    rectifiedCameraInfo->header.frame_id = cameraFrame;
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

  requestDataServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(RequestDataMono)
}

void MonoCamera::onSetParameter(ensenso_camera_msgs::SetParameterGoalConstPtr const& goal)
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

  // Synchronize to make sure that we read back the correct values.
  NxLibCommand synchronize(cmdSynchronize, serial);
  synchronize.parameters()[itmCameras] = serial;
  synchronize.execute();

  saveParameterSet(goal->parameter_set);

  // Calibration could have changed after setting new parameters
  updateCameraInfo();

  // Read back the actual values.
  for (auto const& parameter : goal->parameters)
  {
    result.results.push_back(*readParameter(parameter.key));
  }

  setParameterServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(SetParameter)
}

void MonoCamera::onLocatePattern(ensenso_camera_msgs::LocatePatternMonoGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(LocatePatternMono, locatePatternServer)

  ensenso_camera_msgs::LocatePatternMonoResult result;
  ensenso_camera_msgs::LocatePatternMonoFeedback feedback;

  loadParameterSet(goal->parameter_set);

  int numberOfShots = goal->number_of_shots;
  if (numberOfShots < 1)
  {
    numberOfShots = 1;
  }

  std::vector<MonoCalibrationPattern> patterns;
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
  result.mono_patterns.resize(patterns.size());
  for (size_t i = 0; i < patterns.size(); i++)
  {
    patterns[i].writeToMessage(result.mono_patterns[i]);
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

    result.mono_pattern_poses.resize(patternPoses.size());
    for (size_t i = 0; i < patternPoses.size(); i++)
    {
      result.mono_pattern_poses[i] = stampedPoseFromTransform(patternPoses[i]);
    }
  }
  else
  {
    // Estimate the pose of a single pattern, averaging over the different
    // shots.
    auto patternPose = estimatePatternPose(imageTimestamp, patternFrame);

    result.mono_pattern_poses.resize(1);
    result.mono_pattern_poses[0] = stampedPoseFromTransform(patternPose);
  }

  if (!goal->tf_frame.empty())
  {
    if (patterns.size() == 1)
    {
      geometry_msgs::TransformStamped transform = transformFromPose(result.mono_pattern_poses[0], goal->tf_frame);
      transformBroadcaster->sendTransform(transform);
    }
    else
    {
      ROS_WARN("Cannot publish the pattern pose in TF, because there are "
               "multiple patterns!");
    }
  }

  locatePatternServer->setSucceeded(result);

  FINISH_NXLIB_ACTION(LocatePatternMono)
}

std::vector<MonoCalibrationPattern> MonoCamera::collectPattern(bool clearBuffer) const
{
  if (clearBuffer)
  {
    NxLibCommand(cmdDiscardPatterns, serial).execute();
  }

  NxLibCommand collectPattern(cmdCollectPattern, serial);
  collectPattern.parameters()[itmCameras] = serial;
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

  for (int i = 0; i < collectPattern.result()[itmPatterns][0][serial].count(); i++)
  {
    result.emplace_back(collectPattern.result()[itmPatterns][0][serial][i][itmPattern]);
  }

  // Extract the pattern's image points from the result.
  NxLibItem pattern = collectPattern.result()[itmPatterns][0][serial];

  for (size_t i = 0; i < result.size(); i++)
  {
    for (int j = 0; j < pattern[i][itmPoints].count(); j++)
    {
      NxLibItem pointNode = pattern[i][itmPoints][j];

      ensenso_camera_msgs::ImagePoint point;
      point.x = pointNode[0].asDouble();
      point.y = pointNode[1].asDouble();
      result[i].points.push_back(point);
    }
  }

  return result;
}

geometry_msgs::TransformStamped MonoCamera::estimatePatternPose(ros::Time imageTimestamp,
                                                                std::string const& targetFrame,
                                                                bool latestPatternOnly) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, serial);
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

  ROS_ASSERT(estimatePatternPose.result()[itmPatterns].count() == 1);

  return poseFromNxLib(estimatePatternPose.result()[itmPatterns][0][itmPatternPose], cameraFrame, targetFrame);
}

std::vector<geometry_msgs::TransformStamped> MonoCamera::estimatePatternPoses(ros::Time imageTimestamp,
                                                                              std::string const& targetFrame) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, serial);
  estimatePatternPose.parameters()[itmAverage] = false;
  estimatePatternPose.parameters()[itmType] = itmMonocular;
  estimatePatternPose.parameters()[itmFilter][itmCameras] = serial;

  estimatePatternPose.execute();

  int numberOfPatterns = estimatePatternPose.result()[itmPatterns].count();

  std::vector<geometry_msgs::TransformStamped> result;
  result.reserve(numberOfPatterns);

  for (int i = 0; i < numberOfPatterns; i++)
  {
    result.push_back(
        poseFromNxLib(estimatePatternPose.result()[itmPatterns][i][itmPatternPose], targetFrame, cameraFrame));
  }

  return result;
}
