string type

# Constant types of primitives
string SPHERE = Sphere
string CYLINDER = Cylinder
string PLANE = Plane

# Depending of the type, different parameters are going to be used.
# all types
geometry_msgs/Point32 center
# type = type_plane
geometry_msgs/Point32 normal
# type = type_plane
geometry_msgs/Point32[] axes
# type = type_cylinder
geometry_msgs/Point32 axis
# type = type_sphere or type = type_cylinder
float64 radius

## Primitive parameters
## Input Parameters
# If given, these two following parameters will overwrite the global inlier_fraction and inlier_threshold.
# estimated inlier fraction of the primitive to be found
float64 inlier_fraction_in
float64 inlier_threshold

# Defines the Radius of the Sphere or Cylinder primitive
float64 min_radius
float64 max_radius

# Specifies how much primitives with the same parameters defined should be fitted. Defaults to one.
int32 count

## Result Parameters
float64 score
int32 ransac_iterations
int32 inlier_count
# inlier fraction of the found primitive
float64 inlier_fraction_out

