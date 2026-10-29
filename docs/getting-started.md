# Getting Started

Please refer to our [ROS1 wiki](http://wiki.ros.org/ensenso_driver) for documentation, as there is no documentation platform for third-party ROS2 packages yet. We have changed neither the API nor the message and action definitions for ROS2, so the wiki also applies to the ROS2 driver.

We still provide the same nodes as for ROS1, but one thing that has changed from ROS1 to ROS2 is that there are no nodelets in ROS2 (the concept has been replaced by that of a `Component`), which is why the launch file names have changed and some launch files have even been removed. Please keep this in mind when doing the [tutorials](http://wiki.ros.org/ensenso_driver/Tutorials) on nodelets.

## Install external dependencies

Before we can build the ament workspace we have to install an Ensenso SDK and some external dependencies. If you do not have an Ensenso SDK installed, you can run:

```bash
export ENSENSO_INSTALL=/opt/ensenso
export ENSENSO_SDK_VERSION=<ensenso-sdk-version>

# Omit the next lines if you have ROS2 sourced, otherwise
# replace <your-ros-distro> (e.g. with "humble").
export ROS_VERSION=2
export ROS_DISTRO=<your-ros-distro>

# Install the dependencies.
./.github/scripts/install_external_dependencies.sh
```

If you already have an Ensenso SDK installed, then you can simply run the following commands:

```bash
sudo apt-get -y install libopencv-dev python3-opencv
sudo apt-get -y install ros-${ROS_DISTRO}-tf-transformations
sudo pip3 install transforms3d
```

If you wish to use our xacro based launch files, you have to install these additional dependencies:

```bash
sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui
sudo apt install ros-${ROS_DISTRO}-xacro
```

## Build the package

Assuming you have a workspace folder in which you cloned or linked our `ros_driver`, you should be able to run `colcon build` in the workspace.

## Run the nodes and scripts

After the build you should be able to launch the ensenso camera nodes and run the provided Python scripts. The usage is basically the same as for ROS1 except of course for the different ROS2 CLI:

```bash
source <path-to-your-workspace>/install/setup.bash

# Type the following to see the installed scripts and nodes
ros2 run ensenso_camera <tab><tab>

# Type the following to see the installed launch files
ros2 launch ensenso_camera <tab><tab>

```

## Limitations

Since ROS2 does not support type masquerading (yet and probably never), point clouds in ROS2 are published as `sensor_msgs::msg::PointCloud2` and not directly as `pcl::PointCloud<T>` messages as in ROS1. This requires the user to convert the received point cloud in case `pcl` format is desired.

For more information see:

- https://twitter.com/therealfergs/status/1300128818137649154 (see the comments by Sean Kelly)
- https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
- https://discourse.ros.org/t/optimized-ros-pcl-conversion/25833
