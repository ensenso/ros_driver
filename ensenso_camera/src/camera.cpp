#include "ensenso_camera/camera.h"

#include "ensenso_camera/conversion.h"
#include "ensenso_camera/parameters.h"
#include "ensenso_camera/pose_utilities.h"

#include "ensenso_camera/ros2/tf2.h"

#include "ensenso_camera/ros2/sensor_msgs/distortion_models.h"
#include "ensenso_camera/ros2/diagnostic_msgs/diagnostic_status.h"

ParameterSet::ParameterSet(std::string const& name, NxLibItem const& defaultParameters)
{
  // Create a new NxLib node where we will store the parameters for this set and overwrite it with the default settings.
  node = NxLibItem()["rosParameterSets"][name];
  node << defaultParameters;
}

CameraParameters::CameraParameters(ensenso::ros::NodeHandle& nh, std::string const& cameraType, std::string serial)
  : serial(std::move(serial))
{
  ensenso::ros::get_parameter(nh, "file_camera_path", fileCameraPath);
  isFileCamera = !fileCameraPath.empty();

  ensenso::ros::get_parameter(nh, "fixed", fixed);
  ensenso::ros::get_parameter(nh, "wait_for_camera", waitForCamera);

  ensenso::ros::get_parameter(nh, "camera_frame", cameraFrame);
  if (cameraFrame.empty())
  {
    cameraFrame = "optical_frame_" + this->serial;
  }

  ensenso::ros::get_parameter(nh, "link_frame", linkFrame);
  ensenso::ros::get_parameter(nh, "target_frame", targetFrame);

  if (linkFrame.empty() && !targetFrame.empty())
  {
    linkFrame = targetFrame;
  }
  else if (linkFrame.empty() && targetFrame.empty())
  {
    linkFrame = cameraFrame;
    targetFrame = cameraFrame;
  }
  else if (!linkFrame.empty() && targetFrame.empty())
  {
    targetFrame = linkFrame;
  }

  ensenso::ros::get_parameter(nh, "capture_timeout", captureTimeout);

  if (cameraType != valMonocular)
  {
    ensenso::ros::get_parameter(nh, "robot_frame", robotFrame);
    ensenso::ros::get_parameter(nh, "wrist_frame", wristFrame);

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

    // Load virtual objects and create the handler.
    std::string objectsFile = "";
    ensenso::ros::get_parameter(nh, "objects_file", objectsFile);
    if (!objectsFile.empty())
    {
      // Get objects frame, default to target.
      std::string objectsFrame = targetFrame;
      ensenso::ros::get_parameter(nh, "objects_frame", objectsFrame);

      // Get virtual object marker publish settings.
      // Default to empty topic, meaning no markers are published.
      std::string markerTopic;
      double markerRate = 1;
      ensenso::ros::get_parameter(nh, "visualization_marker_topic", markerTopic);
      ensenso::ros::get_parameter(nh, "visualization_marker_rate", markerRate);

      ENSENSO_DEBUG(nh, "Loading virtual objects...");
      try
      {
        virtualObjectHandler = ensenso::std::make_unique<ensenso_camera::VirtualObjectHandler>(
            nh, objectsFile, objectsFrame, linkFrame, markerTopic, markerRate);
      }
      catch (std::exception const& e)
      {
        ENSENSO_WARN(nh, "Unable to load virtual objects file '%s'. Error: %s", objectsFile.c_str(), e.what());
      }
    }
  }
}

Camera::Camera(ensenso::ros::NodeHandle& nh, CameraParameters _params) : params(std::move(_params)), nh(nh)
{
  tfBuffer = make_tf2_buffer(nh);
  transformListener = ensenso::std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
  transformBroadcaster = make_tf2_broadcaster(nh);

  accessTreeServer = MAKE_SERVER(Camera, AccessTree, "access_tree");
  executeCommandServer = MAKE_SERVER(Camera, ExecuteCommand, "execute_command");
  getParameterServer = MAKE_SERVER(Camera, GetParameter, "get_parameter");
  setParameterServer = MAKE_SERVER(Camera, SetParameter, "set_parameter");

  statusPublisher = ensenso::ros::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(nh, "/diagnostics", 1);

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

bool Camera::loadSettings(std::string const& jsonFile, bool saveAsDefaultParameters)
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
      ENSENSO_ERROR(nh, "The file '%s' does not contain valid JSON", jsonFile.c_str());
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

      ENSENSO_INFO(nh, "Loaded settings from '%s'.", jsonFile.c_str());
    }
    catch (NxLibException& e)
    {
      LOG_NXLIB_EXCEPTION(e)
      return false;
    }
  }
  else
  {
    ENSENSO_ERROR(nh, "Could not open the file '%s'", jsonFile.c_str());
    return false;
  }

  return true;
}

void Camera::onAccessTree(ensenso::action::AccessTreeGoalConstPtr const& goal)
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

  ensenso::action::AccessTreeResult result;

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

void Camera::onExecuteCommand(ensenso::action::ExecuteCommandGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(ExecuteCommand, executeCommandServer)

  loadParameterSet(goal->parameter_set);

  NxLibCommand command(goal->command);
  command.parameters().setJson(goal->parameters);
  command.execute();

  ensenso::action::ExecuteCommandResult result;
  result.result = command.result().asJson();

  executeCommandServer->setSucceeded(std::move(result));

  FINISH_NXLIB_ACTION(ExecuteCommand)
}

void Camera::onGetParameter(ensenso::action::GetParameterGoalConstPtr const& goal)
{
  START_NXLIB_ACTION(GetParameter, getParameterServer)

  ensenso::action::GetParameterResult result;

  loadParameterSet(goal->parameter_set);

  result.stamp = ensenso::ros::now(nh);
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
  return !isIdentity(transformFromNxLib(cameraNode[itmLink]));
}

void Camera::publishStatus(TIMER_CALLBACK_DEFINITION_ARGS)
{
  std::lock_guard<std::mutex> lock(nxLibMutex);

  diagnostic_msgs::msg::DiagnosticStatus cameraStatus;
  cameraStatus.name = "Camera";
  cameraStatus.hardware_id = params.serial;
  cameraStatus.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  cameraStatus.message = "OK";

  if (!cameraIsOpen())
  {
    cameraStatus.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    cameraStatus.message = "Camera is no longer open";
  }

  diagnostic_msgs::msg::DiagnosticArray status;
  status.header.stamp = ensenso::ros::now(nh);
  status.status.push_back(cameraStatus);
  statusPublisher->publish(status);
}

void Camera::fillBasicCameraInfoFromNxLib(sensor_msgs::msg::CameraInfoPtr const& info) const
{
  info->header.frame_id = params.cameraFrame;

  info->width = cameraNode[itmSensor][itmSize][0].asInt();
  info->height = cameraNode[itmSensor][itmSize][1].asInt();

  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  GET_D_MATRIX(info).clear();

  GET_K_MATRIX(info).fill(0);
  GET_P_MATRIX(info).fill(0);
  GET_R_MATRIX(info).fill(0);

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
  writeTransformToNxLib(targetFrameTransformation, NxLibItem()[itmLinks][getNxLibTargetFrameName()]);
}

void Camera::updateGlobalLink(ensenso::ros::Time time, std::string targetFrame, bool useCachedTransformation) const
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
  geometry_msgs::msg::TransformStamped transformMsg;
  if (useCachedTransformation && transformationCache.count(targetFrame) != 0)
  {
    transformMsg = transformationCache[targetFrame];
  }
  else
  {
    transformMsg = tfBuffer->lookupTransform(params.linkFrame, targetFrame, time,
                                             ensenso::ros::durationFromSeconds(TF_REQUEST_TIMEOUT));
    transformationCache[targetFrame] = transformMsg;
  }
  tf2::Transform transform;
  tf2::convert(transformMsg.transform, transform);
  NxLibItem()[itmLinks][getNxLibTargetFrameName()].setNull();
  NxLibItem()[itmLinks].setNull();
  writeTransformToNxLib(transform, NxLibItem()[itmLinks][getNxLibTargetFrameName()]);
}

ensenso::msg::ParameterPtr Camera::readParameter(std::string const& key) const
{
  auto message = ensenso::std::make_shared<ensenso::msg::Parameter>();
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
      ENSENSO_WARN(nh, "Reading the parameter %s, but the camera does not support it!", key.c_str());
    }
  }
  else
  {
    ENSENSO_ERROR(nh, "Unknown parameter '%s'!", key.c_str());
  }

  return message;
}

void Camera::writeParameter(ensenso::msg::Parameter const& parameter)
{
  if (parameterExists(parameter.key))
  {
    // The parameter is mapped to an NxLib node.

    ParameterMapping parameterMapping = parameterInformation.at(parameter.key);
    NxLibItem node = parameterMapping.node(cameraNode);

    if (!node.exists())
    {
      ENSENSO_WARN(nh, "Writing the parameter %s, but the camera does not support it!", parameter.key.c_str());
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
    ENSENSO_ERROR(nh, "Unknown parameter '%s'!", parameter.key.c_str());
  }
}

bool Camera::open()
{
  if (params.isFileCamera && !cameraNode.exists())
  {
    try
    {
      if (params.serial.empty())
      {
        ENSENSO_ERROR(nh, "The serial is empty, please proivde a valid one!");
        return false;
      }
      if (params.serial.size() > 15)
      {
        ENSENSO_ERROR(nh, "The serial '%s' is too long!", params.serial.c_str());
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
      ENSENSO_ERROR(nh, "Failed to create the file camera!");
      LOG_NXLIB_EXCEPTION(e)
      return false;
    }
  }

  if (!cameraNode.exists())
  {
    ENSENSO_ERROR(nh, "The camera '%s' could not be found", params.serial.c_str());
    return false;
  }

  if (params.waitForCamera)
  {
    while (!cameraIsAvailable() && ensenso::ros::ok())
    {
      ENSENSO_INFO(nh, "Waiting for camera '%s' to become available", params.serial.c_str());
      ensenso::ros::sleep(0.5);
    }
  }
  else if (!cameraIsAvailable())
  {
    ENSENSO_ERROR(nh, "The camera '%s' is already in use", params.serial.c_str());
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
    ENSENSO_ERROR(nh, "Error while opening the camera '%s'!", params.serial.c_str());
    LOG_NXLIB_EXCEPTION(e)
    return false;
  }

  saveDefaultParameterSet();
  publishCurrentLinks();
  updateCameraInfo();

  ENSENSO_INFO(nh, "Opened camera with serial number '%s'.", params.serial.c_str());

  if (!nxLibVersion.meetsMinimumRequirement(3, 0))
  {
    ENSENSO_WARN_ONCE(nh, "Ensenso SDK 3.0 or newer is required, you are using version %s.",
                      nxLibVersion.toString().c_str());
  }

  if (hasLink() && params.cameraFrame == params.targetFrame)
  {
    ENSENSO_WARN_ONCE(nh,
                      "Camera %s has an internal link (i.e. it is either extrinsically calibrated (workspace- or "
                      "hand-eye) or has a link to another camera), but camera and target frame are equal, which means "
                      "that neither a link nor a target frame has been provided. The images and 3d data retreived from "
                      "the camera are transformed by the NxLib with the transform stored in the camera's link node, "
                      "however, this transform is not known to tf. Please provide a link or target frame in order for "
                      "the transform to be published.",
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

  ENSENSO_DEBUG(nh, "Loading parameter set '%s'", name.c_str());

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

void Camera::publishCurrentLinks(TIMER_CALLBACK_DEFINITION_ARGS)
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

geometry_msgs::msg::TransformStamped Camera::stampedLinkToCamera()
{
  // Get the inverse of the camera to link transform, because we want to publish the other way around (e.g. instead of
  // camera->link, we want link->camera).
  tf2::Transform cameraToLinkInverse = getCameraToLinkTransform().inverse();
  // The camera always needs to be the child frame in this transformation.
  return fromTf(cameraToLinkInverse, params.linkFrame, params.cameraFrame, ensenso::ros::now(nh));
}

tf2::Transform Camera::getCameraToLinkTransform()
{
  // The NxLib will always give the transform from the camera to the link target in camera coordinates.
  // Always initialize transform otherwise the transform will be invalid.
  tf2::Transform transform = tf2::Transform::getIdentity();

  try
  {
    transform = transformFromNxLib(cameraNode[itmLink]);
  }
  catch (NxLibException const& e)
  {
    ENSENSO_WARN(nh, "Link does not exist. Therefore we cannot publish a transform to any target. Error message: %s",
                 e.getErrorText().c_str());
  }

  if (!isValid(transform))
  {
    ENSENSO_WARN(nh, "Did not find a good transform from %s to %s. Transform has been set to identity",
                 params.cameraFrame.c_str(), params.linkFrame.c_str());
  }

  return transform;
}

void Camera::initTfPublishTimer()
{
  cameraPosePublisher = CREATE_TIMER(nh, POSE_TF_INTERVAL, &Camera::publishCurrentLinks, this);
}

void Camera::initStatusTimer()
{
  statusTimer = CREATE_TIMER(nh, STATUS_INTERVAL, &Camera::publishStatus, this);
}
