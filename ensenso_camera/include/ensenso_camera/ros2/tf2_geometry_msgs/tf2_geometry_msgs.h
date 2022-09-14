#pragma once

#ifdef ROS2
// We use the deprecated .h header instead of .hpp, because the latter is not available for foxy
// TODO Use .hpp header as soon as we do not support foxy anymore
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
