#pragma once

#ifdef ROS2
// We use the deprecated .h header instead of .hpp, because the latter is not available for foxy and humble
// TODO Use .hpp header as soon as we do not support foxy and humble anymore
#ifdef ROS2_JAZZY
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
