# New features

# Bug fixes

* The camera instrinsics matrix is changed to the values of the dynamic calibration when capturing the first image. Therefore, when requesting the camera info it is this matrix that should be returned and not the "static calibration matrix". 

# Migration guide

# Misc