# ROS2
The ROS2 packages in this repo are ignored by default and do not yet contain the C++ and Python source files. Only files
that cannot be maintained in the ROS1 packages, because their structure is too different (e.g. CMakeLists) or there is
no way to provide a compatibility layer between ROS1 and ROS2 (e.g. ensenso_camera_msgs2/action/GetParameter.action,
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
# Omit the next lines if you have ROS2 sourced, otherwise replace <your-ros-distro> (e.g. with "humble").
export ROS_VERSION=2
export ROS_DISTRO=<your-ros-distro>
# Install the dependencies.
./.github/scripts/install_external_dependencies.sh
```

If you already have an Ensenso SDK installed, then you can simply run the following commands:
```
sudo apt-get -y install libopencv-dev python3-opencv
sudo apt-get -y install ros-${ROS_DISTRO}-tf-transformations
sudo pip3 install transforms3d
```

If you wish to use our xacro based launch files, you have to install these additional dependencies:
```
sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui
sudo apt install ros-${ROS_DISTRO}-xacro
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

## Documentation
The usage is basically the same as for ROS1 except of course for the different ROS2 CLI as shown above.

There is no documentation platform for third-party packages for ROS2 yet, but since we neither changed the API nor the
message and action definitions for ROS2, you can simply refer to our [ROS1 wiki](http://wiki.ros.org/ensenso_driver).

We still provide the same nodes as for ROS1, but one thing that has changed from ROS1 to ROS2 is that there are no
nodelets in ROS2 (the concept has been replaced by that of a `Component`), which is why the launch file names have
changed and some launch files have even been removed. Please keep this in mind when doing the
[tutorials](http://wiki.ros.org/ensenso_driver/Tutorials) on nodelets.

## Limitations
Since ROS2 does not support type masquerading (yet and probably never), point clouds in ROS2 are published as
`sensor_msgs::msg::PointCloud2` and not directly as `pcl::PointCloud<T>` messages as in ROS1. This requires the user to
convert the received point cloud in case `pcl` format is desired.

For more information see:
* https://twitter.com/therealfergs/status/1300128818137649154 (see the comments by Sean Kelly)
* https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
* https://discourse.ros.org/t/optimized-ros-pcl-conversion/25833