#!/bin/bash

# Install script for external dependencies such as the Esenso SDK.

set -e

# Requires ENSENSO_INSTALL and ENSENSO_SDK_VERSION to be set.
sudo apt-get -y install dpkg wget
wget -O /tmp/ensenso.deb https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-${ENSENSO_SDK_VERSION}-x64.deb
sudo dpkg -i /tmp/ensenso.deb
sudo apt-get install -f -y
sudo pip3 install numpy==1.22.4 scipy==1.10.0

if [[ $ROS_VERSION -eq "2" ]]; then
    sudo apt-get -y install libopencv-dev python3-opencv
    sudo apt-get -y install ros-$ROS_DISTRO-tf-transformations
    sudo pip3 install transforms3d
fi