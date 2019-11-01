# Release v1.6.2

## Bug fixes

* When converting a pointcloud to rgbd, the reprojection was fixed such that the final depth image contains the value of the point closest to the camera.

# Release v1.6.0

## New features

* A leveled camera is published to the static tf. This camera is located at the exact same position as the opened camera, but parallel to the world xy plane

* When requested, a depth map from the leveld camera perspective is published

## Bug fixes

## Migration guide

## Misc