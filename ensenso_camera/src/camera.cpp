#include "ensenso_camera/camera.h"

#include "ensenso_camera/parameters.h"
#include "ensenso_camera/pose_utilities.h"
#include "ensenso_camera/conversion.h"

#include <sensor_msgs/distortion_models.h>

ParameterSet::ParameterSet(const std::string& name, const NxLibItem& defaultParameters)
{
  // Create a new NxLib node where we will store the parameters for this set and
  // overwrite it with the default settings.
  node = NxLibItem()["rosParameterSets"][name];
  node << defaultParameters;
}

Camera::Camera(ros::NodeHandle const& n, std::string serial, std::string fileCameraPath, bool fixed,
               std::string cameraFrame, std::string targetFrame, std::string linkFrame)
  : fileCameraPath(std::move(fileCameraPath))
  , serial(std::move(serial))
  , fixed(fixed)
  , cameraFrame(std::move(cameraFrame))
  , linkFrame(std::move(linkFrame))
  , targetFrame(std::move(targetFrame))
  , nh(n)
{
  transformListener = make_unique<tf2_ros::TransformListener>(tfBuffer);
  transformBroadcaster = make_unique<tf2_ros::TransformBroadcaster>();

  isFileCamera = !this->fileCameraPath.empty();

  accessTreeServer = ::make_unique<AccessTreeServer>(nh, "access_tree", boost::bind(&Camera::onAccessTree, this, _1));
  executeCommandServer =
      ::make_unique<ExecuteCommandServer>(nh, "execute_command", boost::bind(&Camera::onExecuteCommand, this, _1));
  getParameterServer =
      ::make_unique<GetParameterServer>(nh, "get_parameter", boost::bind(&Camera::onGetParameter, this, _1));
  setParameterServer =
      ::make_unique<SetParameterServer>(nh, "set_parameter", boost::bind(&Camera::onSetParameter, this, _1));

  statusPublisher = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

  cameraNode = NxLibItem()[itmCameras][itmBySerialNo][this->serial];
  nxLibVersion = getCurrentNxLibVersion();

  defaultParameters = NxLibItem()["rosDefaultParameters"];
}

void Camera::startServers() const
{
  accessTreeServer->start();
  executeCommandServer->start();

  getParameterServer->start();
  setParameterServer->start();
}

bool Camera::loadSettings(const std::string& jsonFile, bool saveAsDefaultParameters)
{
  if (jsonFile.empty())
    return true;

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

      NxLibCommand synchronize(cmdSynchronize, serial);
      synchronize.parameters()[itmCameras] = serial;
      synchronize.execute();

      updateCameraInfo();
      if (saveAsDefaultParameters)
        saveDefaultParameterSet();
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

  loadParameterSet(goal->parameter_set);

  NxLibItem item(goal->path);

  bool treeChanged = false;
  if (goal->erase)
  {
    item.erase();
    treeChanged = true;
  }
  else if (goal->set_null)
  {
    item.setNull();
    treeChanged = true;
  }
  else if (!goal->json_value.empty())
  {
    item.setJson(goal->json_value);
    treeChanged = true;
  }

  if (treeChanged)
  {
    saveParameterSet(goal->parameter_set);
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

  loadParameterSet(goal->parameter_set);

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

void Camera::fillBasicCameraInfoFromNxLib(sensor_msgs::CameraInfoPtr const& info) const
{
  info->header.frame_id = cameraFrame;

  info->width = cameraNode[itmSensor][itmSize][0].asInt();
  info->height = cameraNode[itmSensor][itmSize][1].asInt();

  info->binning_x = cameraNode[itmParameters][itmCapture][itmBinning].asInt();
  info->binning_y = info->binning_x;
}

void Camera::updateTransformations(tf2::Transform const& targetFrameTransformation) const
{
  cameraNode[itmLink][itmTarget] = TARGET_FRAME_LINK + "_" + serial;
  writePoseToNxLib(targetFrameTransformation, NxLibItem()[itmLinks][TARGET_FRAME_LINK + "_" + serial]);
}

void Camera::updateGlobalLink(ros::Time time, std::string frame, bool useCachedTransformation) const
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

  cameraNode[itmLink][itmTarget] = TARGET_FRAME_LINK + "_" + serial;

  // Update the transformation to the target frame in the NxLib according to
  // the current information from TF. Only if the link frame and target frame differs. Otherwise the transformation will
  // be doubled. If they're the same, the global Link must be the identity transform and the tf will be in the camera
  // link. (camera -> link in Nxlib or link -> camera in ROS).
  if (frame != linkFrame)
  {
    geometry_msgs::TransformStamped transform;
    if (useCachedTransformation && transformationCache.count(frame) != 0)
    {
      transform = transformationCache[frame];
    }
    else
    {
      transform = tfBuffer.lookupTransform(linkFrame, frame, time, ros::Duration(TRANSFORMATION_REQUEST_TIMEOUT));
      transformationCache[frame] = transform;
    }
    tf2::Transform tfTrafo;
    tf2::convert(transform.transform, tfTrafo);
    NxLibItem()[itmLinks][TARGET_FRAME_LINK + "_" + serial].setNull();
    NxLibItem()[itmLinks].setNull();
    writePoseToNxLib(tfTrafo, NxLibItem()[itmLinks][TARGET_FRAME_LINK + "_" + serial]);
  }
}

std::vector<geometry_msgs::TransformStamped> Camera::estimatePatternPoses(ros::Time imageTimestamp,
                                                                          std::string const& targetFrame) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, serial);
  estimatePatternPose.parameters()[itmAverage] = false;
  estimatePatternPose.parameters()[itmFilter][itmCameras] = serial;
  estimatePatternPose.parameters()[itmFilter][itmUseModel] = true;
  estimatePatternPose.parameters()[itmFilter][itmType] = valStatic;
  estimatePatternPose.parameters()[itmFilter][itmValue] = true;
  estimatePatternPose.execute();

  int numberOfPatterns = estimatePatternPose.result()[itmPatterns].count();

  std::vector<geometry_msgs::TransformStamped> result;
  result.reserve(numberOfPatterns);

  for (int i = 0; i < numberOfPatterns; i++)
  {
    result.push_back(
        poseFromNxLib(estimatePatternPose.result()[itmPatterns][i][itmPatternPose], cameraFrame, targetFrame));
  }

  return result;
}

geometry_msgs::TransformStamped Camera::estimatePatternPose(ros::Time imageTimestamp, std::string const& targetFrame,
                                                            bool latestPatternOnly) const
{
  updateGlobalLink(imageTimestamp, targetFrame);

  NxLibCommand estimatePatternPose(cmdEstimatePatternPose, serial);
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

ensenso_camera_msgs::ParameterPtr Camera::readParameter(std::string const& key) const
{
  auto message = boost::make_shared<ensenso_camera_msgs::Parameter>();
  message->key = key;
  if (parameterExists(key))
  {
    // The parameter is mapped to an NxLib node.

    ParameterMapping parameterMapping = parameterInformation.at(key);
    NxLibItem node = parameterMapping.node(cameraNode);

    if (node.exists())
    {
      switch (parameterMapping.type)
      {
        case ParameterType::Bool:
          message->bool_value = node.asBool();
          break;
        case ParameterType::Number:
          message->float_value = node.asDouble();
          break;
        case ParameterType::String:
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
  if (parameterExists(parameter.key))
  {
    // The parameter is mapped to an NxLib node.

    ParameterMapping parameterMapping = parameterInformation.at(parameter.key);
    NxLibItem node = parameterMapping.node(cameraNode);

    if (!node.exists())
    {
      ROS_WARN("Writing the parameter %s, but the camera does not support it!", parameter.key.c_str());
      return;
    }

    switch (parameterMapping.type)
    {
      case ParameterType::Bool:
        node.set<bool>(parameter.bool_value);
        break;
      case ParameterType::Number:
        node.set<double>(parameter.float_value);
        break;
      case ParameterType::String:
        node.set<std::string>(parameter.string_value);
    }
  }
  else
  {
    ROS_ERROR("Unknown parameter '%s'!", parameter.key.c_str());
  }
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

      NxLibCommand createCamera(cmdCreateCamera, serial);
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
    NxLibCommand open(cmdOpen, serial);
    open.parameters()[itmCameras] = serial;
    open.execute();
  }
  catch (NxLibException& e)
  {
    ROS_ERROR("Error while opening the camera '%s'!", serial.c_str());
    LOG_NXLIB_EXCEPTION(e)
    return false;
  }

  saveDefaultParameterSet();
  publishCurrentLinks();

  ROS_INFO("Opened camera with serial number '%s'.", serial.c_str());
  return true;
}

void Camera::close()
{
  std::lock_guard<std::mutex> lock(nxLibMutex);

  try
  {
    NxLibCommand close(cmdClose, serial);
    close.parameters()[itmCameras] = serial;
    close.execute();

    if (isFileCamera && createdFileCamera)
    {
      NxLibCommand deleteCmd(cmdDeleteCamera, serial);
      deleteCmd.parameters()[itmCameras] = serial;
      deleteCmd.execute();
    }
  }
  catch (NxLibException&)
  {
  }
}

void Camera::saveDefaultParameterSet()
{
  defaultParameters << cameraNode[itmParameters];
}

void Camera::saveParameterSet(std::string name)
{
  if (name.empty())
  {
    name = DEFAULT_PARAMETER_SET;
  }

  ParameterSet& parameterSet = parameterSets.at(name);

  parameterSet.node.erase();
  parameterSet.node << cameraNode[itmParameters];
}

void Camera::loadParameterSet(std::string name)
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

  if (changedParameters)
  {
    NxLibCommand synchronize(cmdSynchronize, serial);
    synchronize.parameters()[itmCameras] = serial;
    synchronize.execute();
  }

  currentParameterSet = name;
}

void Camera::publishCurrentLinks(ros::TimerEvent const& timerEvent)
{
  publishCameraLink();
}

void Camera::publishCameraLink()
{
  // The camera link is the calibrated link from camera to link
  if (cameraFrame == linkFrame)
  {
    return;
  }

  bool cameraLinkExists = cameraNode[itmLink][itmTarget].exists();

  if (cameraLinkExists)
  {
    transformBroadcaster->sendTransform(stampedLinkToCamera());
  }
}

geometry_msgs::TransformStamped Camera::stampedLinkToCamera()
{
  tf2::Transform transform = getCameraToLinkTransform();
  // Need the inverse, because we want to publish the other way around
  // E.g. Instead of camera->link, we want link->camera
  // We also need the camera always to be the child frame
  geometry_msgs::TransformStamped transformStamped = fromTfTransform(transform.inverse(), linkFrame, cameraFrame);
  return transformStamped;
}

tf2::Transform Camera::getCameraToLinkTransform()
{
  // The Nxlib will always give the transfrom from the Camera to the Link target in Camera Coordinates.
  tf2::Transform transform;
  try
  {
    transform = poseFromNxLib(cameraNode[itmLink]);
  }
  catch (NxLibException const& e)
  {
    ROS_WARN("Link does not exists.Therefore we cannot publish a transform to any target. Error message: %s",
             e.getErrorText().c_str());
  }

  if (!isValid(transform))
  {
    transform.setIdentity();
    ROS_WARN("Did not find a good transform from %s to %s. Transform has been set to identity", cameraFrame.c_str(),
             linkFrame.c_str());
  }

  return transform;
}

void Camera::initTfPublishTimer()
{
  cameraPosePublisher = nh.createTimer(ros::Duration(POSE_TF_INTERVAL), &Camera::publishCurrentLinks, this);
}

void Camera::initStatusTimer()
{
  statusTimer = nh.createTimer(ros::Duration(STATUS_INTERVAL), &Camera::publishStatus, this);
}
