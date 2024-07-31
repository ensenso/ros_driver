# Devcontainer for ROS1 and ROS2

- Make sure you have docker and the VSCode extension "Dev Containers" installed
- Hit ctrl + shift + P and type "Dev Containers: Reopen in Container"
- Select the container with the desired ROS and Ensenso SDK version
- Upon opening, the container automatically sets up and builds the workspace
- From within every container, you can run `./test.sh` from the workspace folder
- The script lets you select one of the available tests
- For any other ROS command, the ROS overlay and the workspace overlay have to be sourced first
- This can be done by running `source overlay.sh`
- In case of ROS2, the repository structure is changed such that it can be built with ROS2; the corresponding changes are committed as well as any pre-existing changes in the working tree before preparing the ROS2 build
