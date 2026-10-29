# ROS1

The ROS1 packages in this repo are ignored by default and do not yet contain the C++ and Python source files. Only files that cannot be maintained in the ROS2 packages, because their structure is too different (e.g. CMakeLists) or there is no way to provide a compatibility layer between ROS1 and ROS2 (e.g. ensenso_camera_msgs2/action/GetParameter.action, here the `Time` interface changed), are already present in the ROS1 packages.

## Prepare the repo

We provide a script that takes care of ignoring all ROS2 packages and recognizing all ROS1 packages within this repo as well as copying the C++ and Python and all other relevant files into the ROS1 packages.

Execute the script as follows in order to prepare your ROS1 build:

```bash
# Assuming you have a catkin workspace in your home directory
# and the rosdriver has been cloned or linked in the `src` directory
cd ~/catkin_workspace/src/ros_driver

# If ROS1 is sourced
./.github/scripts/prepare_ros1_build.sh

# Or if ROS1 is not sourced
export ROS_VERSION=1 && ./.github/scripts/prepare_ros1_build.sh
```

## Install external dependencies

Install the external dependencies as explained [here](getting-started.md#install-external-dependencies).
