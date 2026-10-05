#!/bin/bash

# Install script for external dependencies such as the Esenso SDK.

set -e

# Requires ENSENSO_INSTALL and ENSENSO_SDK_VERSION to be set.
sudo apt-get -y install dpkg wget
case "$(dpkg --print-architecture)" in
    amd64) ensenso_arch=x64 ;;
    arm64) ensenso_arch=arm64 ;;
    *) echo "Unsupported CPU architecture: $(dpkg --print-architecture)" >&2; exit 1 ;;
esac
wget -O /tmp/ensenso.deb "https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-${ENSENSO_SDK_VERSION}-${ensenso_arch}.deb"
sudo dpkg -i /tmp/ensenso.deb
sudo apt-get install -f -y
rm -f /tmp/ensenso.deb

if [[ $ROS_VERSION -eq "2" ]]; then
    sudo apt-get -y install libpcl-dev libopencv-dev python3-opencv
    sudo apt-get -y install ros-$ROS_DISTRO-tf-transformations
fi