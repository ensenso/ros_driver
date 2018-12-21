# When registered_point_cloud is set, monocular_camera_serial definies the
# serial number of the monocular camera to which the point cloud should be
# registered to, if unset the linked camera of the node is used as reference.
string monocular_camera_serial
# When rendering a point cloud exclude points closer than this parameter.
float64 near
# When rendering a point cloud exclude points further than this parameter.
# if far <= near the defaults near=1 and far=10000 are used.
float64 far
# Use OpenGL for rendering the pointcloud if this flag is set.
bool use_opengl
