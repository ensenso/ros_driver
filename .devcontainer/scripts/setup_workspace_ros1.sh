#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash

cd /home/ws/

echo -e "\n=== Running rosdep update and install =================================================="
mkdir -p log
echo -e "Updating ..."
rosdep update > log/rosdep_update.log
echo -e "Installing ..."
rosdep install -i --from-paths src --rosdistro ${ROS_DISTRO} -y > log/rosdep_install.log

echo -e "\n=== Catkin build ======================================================================="
catkin build

echo -e "\n=== Add helper scripts ================================================================="
echo -e "Add overlay script"
cp src/driver/.devcontainer/scripts/source_overlay_ros1.sh overlay.sh

echo -e "Add test script"
cp src/driver/.devcontainer/scripts/run_test_ros1.sh test.sh
