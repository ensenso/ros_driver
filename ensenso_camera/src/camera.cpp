#include "ensenso_camera/camera.h"

#include "ensenso_camera/conversion.h"
#include "ensenso_camera/parameters.h"
#include "ensenso_camera/pose_utilities.h"

#include <sensor_msgs/distortion_models.h>

#define MAKE_SERVER(TYPE, TAG) ::make_unique<TYPE##Server>(nh, #TAG, boost::bind(&Camera::on##TYPE, this, _1))

ParameterSet::ParameterSet(const std::string& name, const NxLibItem& defaultParameters)
{
  // Create a new NxLib node where we will store the parameters for this set and overwrite it with the default settings.
  node = NxLibItem()["rosParameterSets"][name];
  node << defaultParameters;
}

CameraParameters::CameraParameters(ros::NodeHandle const& nh, std::string const& cameraType, std::string serial)
  : serial(std::move(serial))
{
  nh.param<std::string>("file_camera_path", fileCameraPath, "");
  isFileCamera = !fileCameraPath.empty();

  nh.param("fixed", fixed, false);
  nh.param("wait_for_camera", wait_for_camera, false);

  if (!nh.getParam("camera_frame", cameraFrame))
  {
    cameraFrame = "optical_frame_" + this->serial;
  }

  nh.getParam("link_frame", linkFrame);
  nh.getParam("target_frame", targetFrame);

  if (linkFrame.empty() && !targetFrame.empty())
  {
    linkFrame = targetFrame;
  }
  else if (linkFrame.empty() && targetFrame.empty())
  {
    linkFrame = cameraFrame;
    targetFrame = cameraFrame;
  }

  if (cameraType != valMonocular)
  {
    nh.getParam("robot_frame", robotFrame);
    nh.getParam("wrist_frame", wristFrame);

    if (fixed && robotFrame.empty())
    {
      robotFrame = targetFrame;
    }
    if (!fixed && wristFrame.empty())
    {
      wristFrame = linkFrame;
      if (robotFrame.empty())
      {
        robotFrame = targetFrame;
      }
    }

    nh.param("capture_timeout", captureTimeout, 0);

    // Load virtual objects and create the handler.
    std::string objectsFile;
    if (nh.getParam("objects_file", objectsFile) && !objectsFile.empty())
    {
      // Get objects frame, default to target.
      std::string objectsFrame = targetFrame;
      nh.getParam("objects_frame", objectsFrame);

      ROS_DEBUG("Loading virtual objects...");
      try
      {
        virtualObjectHandler =
            ::make_unique<ensenso_camera::VirtualObjectHandler>(objectsFile, objectsFrame, cameraFrame);
      }
      catch (std::exception const& e)
      {
        ROS_WARN("Unable to load virtual objects file '%s'. Error: %s", objectsFile.c_str(), e.what());
      }
    }
  }
}

Camera::Camera(ros::NodeHandle& nh, CameraParameters _params) : params(std::move(_params)), nh(nh)
{
  tfBuffer = make_unique<tf2_ros::Buffer>();
  transformListener = make_unique<tf2_ros::TransformListener>(*tfBuffer);
  transformBroadcaster = make_unique<tf2_ros::TransformBroadcaster>();

  accessTreeServer = MAKE_SERVER(AccessTree, access_tree);
  executeCommandServer = MAKE_SERVER(ExecuteCommand, execute_command);
  getParameterServer = MAKE_SERVER(GetParameter, get_parameter);
  setParameterServer = MAKE_SERVER(SetParameter, set_parameter);

  statusPublisher = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

  nxLibVersion.fillFromNxLib();

  cameraNode = NxLibItem()[itmCameras][itmBySerialNo][params.serial];
  defaultParameters = NxLibItem()["rosDefaultParameters"][params.serial + "_" + DEFAULT_PARAMETER_SET];
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
  {
    return true;
  }

  std::ifstream file(expandPath(jsonFile));
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

      NxLibCommand synchronize(cmdSynchronize, params.serial);
      synchronize.parameters()[itmCameras] = params.serial;
      synchronize.execute();

      updateCameraInfo();
      if (saveAsDefaultParameters)
      {
        saveDefaultParameterSet();
      }

      ROS_INFO("Loaded settings from '%s'.", jsonFile.c_str());
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
      // This could load any image that does not belong to the node's camera, so we do not know its frame.
      result.binary_data = *imageFromNxLibNode(item, "", params.isFileCamera);
    }
    catch (NxLibException&)
    {
    }  // The item was not binary.
  }

  accessTreeServer->setSucceeded(std::move(result));

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

  executeCommandServer->setSucceeded(std::move(result));

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

  getParameterServer->setSucceeded(std::move(result));

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

bool Camera::hasLink() const
{
  return !isIdentity(poseFromNxLib(cameraNode[itmLink]));
}

void Camera::publishStatus(ros::TimerEvent const& event) const
{
  std::lock_guard<std::mutex> lock(nxLibMutex);

  diagnostic_msgs::DiagnosticStatus cameraStatus;
  cameraStatus.name = "Camera";
  cameraStatus.hardware_id = params.serial;
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
  info->header.frame_id = params.cameraFrame;

  info->width = cameraNode[itmSensor][itmSize][0].asInt();
  info->height = cameraNode[itmSensor][itmSize][1].asInt();

  if (cameraNode[itmParameters][itmCapture][itmBinning].exists())
  {
    info->binning_x = cameraNode[itmParameters][itmCapture][itmBinning].asInt();
    info->binning_y = info->binning_x;
  }
  else
  {
    info->binning_x = 1;
    info->binning_y = 1;
  }
}

std::string Camera::getNxLibTargetFrameName() const
{
  return TARGET_FRAME_LINK + "_" + params.serial;
}

void Camera::updateTransformations(tf2::Transform const& targetFrameTransformation) const
{
  cameraNode[itmLink][itmTarget] = getNxLibTargetFrameName();
  writePoseToNxLib(targetFrameTransformation, NxLibItem()[itmLinks][getNxLibTargetFrameName()]);
}

void Camera::updateGlobalLink(ros::Time time, std::string targetFrame, bool useCachedTransformation) const
{
  // Transformations are represented in the NxLib as follows:
  //  - The camera's link node might contain calibration data from e.g. a hand-eye calibration. This is always used if
  //    is present.
  //  - The transformation between the link frame and the target frame (in which the data is returned) is fetched from
  //    tf and written to the global link node of the NxLib.
  //  - The link in the camera node has to reference this global link if it exists (e.g. if the linkFrame is different
  //    from the targetFrame).

  if (targetFrame.empty())
  {
    targetFrame = params.targetFrame;
  }

  if (params.linkFrame == targetFrame)
  {
    // The given target frame is the target frame already. So the camera does not need a reference to a global link.
    cameraNode[itmLink][itmTarget] = "";
    return;
  }

  cameraNode[itmLink][itmTarget] = getNxLibTargetFrameName();

  // Update the transformation to the target frame in the NxLib according to the current information from tf only if
  // link and target frame differ.
  geometry_msgs::TransformStamped transform;
  if (useCachedTransformation && transformationCache.count(targetFrame) != 0)
  {
    transform = transformationCache[targetFrame];
  }
  else
  {
    transform = tfBuffer->lookupTransform(params.linkFrame, targetFrame, time, ros::Duration(TF_REQUEST_TIMEOUT));
    transformationCache[targetFrame] = transform;
  }
  tf2::Transform tfTrafo;
  tf2::convert(transform.transform, tfTrafo);
  NxLibItem()[itmLinks][getNxLibTargetFrameName()].setNull();
  NxLibItem()[itmLinks].setNull();
  writePoseToNxLib(tfTrafo, NxLibItem()[itmLinks][getNxLibTargetFrameName()]);
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
  if (params.isFileCamera && !cameraNode.exists())
  {
    try
    {
      if (params.serial.size() > 15)
      {
        ROS_ERROR("The serial '%s' is too long!", params.serial.c_str());
        return false;
      }

      NxLibCommand createCamera(cmdCreateCamera, params.serial);
      createCamera.parameters()[itmSerialNumber] = params.serial;
      createCamera.parameters()[itmFolderPath] = params.fileCameraPath;
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
    ROS_ERROR("The camera '%s' could not be found", params.serial.c_str());
    return false;
  }

  if (params.wait_for_camera)
  {
    while (!cameraIsAvailable() && ros::ok())
    {
      ROS_INFO("Waiting for camera '%s' to become available", params.serial.c_str());
      ros::Duration(0.5).sleep();
    }
  }
  else if (!cameraIsAvailable())
  {
    ROS_ERROR("The camera '%s' is already in use", params.serial.c_str());
    return false;
  }

  // At this point the camera does for sure exist (since in case it is a file camera it has been created above) and the
  // camera type is available in the NxLib. Update the camera type in order to also support S-series cameras, which are
  // a subtype of stereo and do not have their own ROS node, but are handled by the stereo node.
  updateCameraTypeSpecifics();

  try
  {
    NxLibCommand open(cmdOpen, params.serial);
    open.parameters()[itmCameras] = params.serial;
    open.execute();
  }
  catch (NxLibException& e)
  {
    ROS_ERROR("Error while opening the camera '%s'!", params.serial.c_str());
    LOG_NXLIB_EXCEPTION(e)
    return false;
  }

  saveDefaultParameterSet();
  publishCurrentLinks();
  updateCameraInfo();

  ROS_INFO("Opened camera with serial number '%s'.", params.serial.c_str());

  if (!nxLibVersion.meetsMinimumRequirement(3, 0))
  {
    ROS_WARN_ONCE("Ensenso SDK 3.0 or newer is required, you are using version %s.", nxLibVersion.toString().c_str());
  }

  if (hasLink() && params.cameraFrame == params.targetFrame)
  {
    ROS_WARN_ONCE(
        "Camera %s has an internal link (i.e. it is either extrinsically calibrated (workspace- or hand-eye) or has a "
        "link to another camera), but camera and target frame are equal, which means that neither a link nor a target "
        "frame has been provided. The images and 3d data retreived from the camera are transformed by the NxLib with "
        "the transform stored in the camera's link node, however, this transform is not known to tf. Please provide "
        "a link or target frame in order for the transform to be published.",
        params.serial.c_str());
  }
  return true;
}

void Camera::close()
{
  std::lock_guard<std::mutex> lock(nxLibMutex);

  try
  {
    NxLibCommand close(cmdClose, params.serial);
    close.parameters()[itmCameras] = params.serial;
    close.execute();

    if (params.isFileCamera && createdFileCamera)
    {
      NxLibCommand deleteCmd(cmdDeleteCamera, params.serial);
      deleteCmd.parameters()[itmCameras] = params.serial;
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
    // The parameter set was never used before. Create it by copying the default settings.
    std::string nodeName = params.serial + "_" + name;
    parameterSets.insert(std::make_pair(name, ParameterSet(nodeName, defaultParameters)));
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
    NxLibCommand synchronize(cmdSynchronize, params.serial);
    synchronize.parameters()[itmCameras] = params.serial;
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
  if (!hasLink() || params.cameraFrame == params.linkFrame)
  {
    return;
  }

  transformBroadcaster->sendTransform(stampedLinkToCamera());
}

geometry_msgs::TransformStamped Camera::stampedLinkToCamera()
{
  // Get the inverse of the camera to link transform, because we want to publish the other way around (e.g. instead of
  // camera->link, we want link->camera).
  tf2::Transform cameraToLinkInverse = getCameraToLinkTransform().inverse();
  // The camera always needs to be the child frame in this transformation.
  return fromTfTransform(cameraToLinkInverse, params.linkFrame, params.cameraFrame);
}

tf2::Transform Camera::getCameraToLinkTransform()
{
  // The NxLib will always give the transform from the camera to the link target in camera coordinates.
  // Always initialize transform otherwise the transform will be invalid.
  tf2::Transform transform = tf2::Transform::getIdentity();

  try
  {
    transform = poseFromNxLib(cameraNode[itmLink]);
  }
  catch (NxLibException const& e)
  {
    ROS_WARN("Link does not exist. Therefore we cannot publish a transform to any target. Error message: %s",
             e.getErrorText().c_str());
  }

  if (!isValid(transform))
  {
    ROS_WARN("Did not find a good transform from %s to %s. Transform has been set to identity",
             params.cameraFrame.c_str(), params.linkFrame.c_str());
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
