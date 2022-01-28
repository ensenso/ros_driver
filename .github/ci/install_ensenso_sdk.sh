#!/bin/bash

# Install script for Ensenso SDK.
# Requires ENSENSO_INSTALL and ENSENSO_SDK_VERSION to be set.

set -e

sudo apt-get -y install dpkg wget
wget -O /tmp/ensenso.deb https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-${ENSENSO_SDK_VERSION}-x64.deb
sudo dpkg -i /tmp/ensenso.deb
sudo apt-get install -f -y