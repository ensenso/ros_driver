float64 thickness     # The thickness of the pattern substrate in meters.
float64 grid_spacing  # The distance between two grid points in meters.
int32 grid_size_x     # The grid size in x and y direction.
int32 grid_size_y

# The pattern points in the left and right image respectively.
# Note that these points are only provided when there is a single pattern in
# both images. Otherwise we cannot tell which of the patterns belong together
# without comparing them with the stereo model.
ImagePoint[] left_points
ImagePoint[] right_points