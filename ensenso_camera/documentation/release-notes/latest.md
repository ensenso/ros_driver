# New features

* Added boolean variable to replace the NaN values of a point cloud with a constant.
* The camera intrinsics of the monocular camera are retrieved as result of the request_data action call.

# Bug fixes

* The camera instrinsics matrix is changed to the values of the dynamic calibration when capturing the first image. Therefore, when requesting the camera info it is this matrix that should be returned and not the "static calibration matrix". 

# Migration guide

# Misc
