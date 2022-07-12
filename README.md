# Official ROS driver for [Ensenso](http://www.ensenso.com) stereo cameras [![Build&Test](https://github.com/ensenso/ros_driver/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/ensenso/ros_driver/actions/workflows/build-and-test.yml)


## Documentation

To get started with the package, take a look at the [ROS wiki](http://wiki.ros.org/ensenso_driver).

For instructions on how to build for ROS2, see [below](#ros2).

## Remarks
 * For using this package, you need to have the [Ensenso SDK](https://www.ensenso.com/support/sdk-download) installed.
 * Version 1.7.0 and newer requires at least Ensenso SDK 3.0.
 * Version 1.6.3 and older requires at least Ensenso SDK 2.0. Older versions are not supported.
 * All inputs and outputs of this package are in meters and seconds, as the convention for ROS requires. This is different from the NxLib, which uses millimeters and milliseconds.

## Report a Bug

Please report any bugs or feature requests in the [issue tracker](https://github.com/ensenso/ros_driver/issues).

## Acknowledgements

<img src="https://raw.githubusercontent.com/ensenso/ros_driver/master/media/rosin.png" alt="ROSIN" title="ROSIN" height="70">&nbsp;<img src="https://raw.githubusercontent.com/ensenso/ros_driver/master/media/eu.png" alt="EU" title="EU" height="70">

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: http://rosin-project.eu/

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement no. 732287.

# ROS2
The ROS2 packages in this repo are ignored by default and do not yet contain the C++ and Python source files. Only files
that cannot be maintained in the ROS1 packages, because their structure is too different (e.g. CMakeLists) or there is
no way to provide a compatability layer between ROS1 and ROS2 (e.g. ensenso_camera_msgs2/action/GetParameter.action,
here the `Time` interface changed), are already present in the ROS2 packages.

## Prepare the repo
We provide a script that takes care of ignoring all ROS1 packages and recognizing all ROS2 packages within this repo as
well as copying the C++ and Python and all other relevant files into the ROS2 packages.

Execute the script as follows in order to prepare your ROS2 build:
```
# Assuming you have an ament workspace in your home directory
# and the rosdriver has been cloned or linked in the `src` directory
cd ~/ament_workspace/src/ros_driver

# If ROS2 is sourced
./.github/scripts/prepare_ros2_build.sh

# Or if ROS2 is not sourced
export ROS_VERSION=2 && ./.github/scripts/prepare_ros2_build.sh
```

## Install external dependencies
Before we can build the ament workspace we have to install an Ensenso SDK and some external dependencies. If you do not
have an Ensenso SDK installed, you can run:
```
export ENSENSO_INSTALL=/opt/ensenso
export ENSENSO_SDK_VERSION=3.3.1385
# Omit this if you have ROS2 sourced
export ROS_VERSION=2
./.github/scripts/install_external_dependencies.sh
```

If you already have an Ensenso SDK installed, then you can simply run the following commands:
```
sudo apt-get -y install libopencv-dev python3-opencv
sudo apt-get -y install ros-galactic-tf-transformations
sudo pip3 install transforms3d
```

## Build the package
Now you should be able to run `colcon build`.

## Run the nodes and scripts
After the build you should be able to launch the ensenso camera nodes and run the provided Python scripts:

```
source ~/ament_workspace/install/setup.bash

# Type the following to see the installed scripts and nodes
ros2 run ensenso_camera <tab><tab>

# Type the following to see the installed launch files
ros2 launch ensenso_camera <tab><tab>

```
