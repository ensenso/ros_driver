#!/bin/bash

cd /home/ws/src/driver

source /opt/ros/${ROS_DISTRO}/setup.bash

echo -e "\n=== Modify codebase and commit changes for ROS2 build =================================="
git config --global user.email "dev@contain.er"
git config --global user.name "Dev Container"

if [[ -n $(git status -s) ]]; then
    git add -A
    echo "Comitting working tree changes ..."
    git commit -m "wip: save changes before ROS2 codebase modification" > /dev/null 2>&1
    git log -1 --oneline
fi

./.github/scripts/prepare_ros2_build.sh 2> /dev/null

if [[ -n $(git status -s) ]]; then
    git add -A
    echo "Comitting created ROS2 files ..."
    git commit -m "tmp: modify codebase for ROS2 build" > /dev/null 2>&1
    git log -1 --oneline
fi

cd ../..
echo -e "\n=== Running rosdep update and install =================================================="
mkdir -p log
echo -e "Updating ..."
rosdep update > log/rosdep_update.log
echo -e "Installing ..."
rosdep install -i --from-paths src --rosdistro ${ROS_DISTRO} -y > log/rosdep_install.log

echo -e "\n=== Colcon build ======================================================================="
colcon build

echo -e "\n=== Add helper scripts ================================================================="
echo -e "Add overlay script"
cp src/driver/.devcontainer/scripts/source_overlay_ros2.sh overlay.sh

echo -e "Add test script"
cp src/driver/.devcontainer/scripts/run_test_ros2.sh test.sh
