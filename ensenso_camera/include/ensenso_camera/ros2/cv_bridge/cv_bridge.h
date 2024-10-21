#pragma once

#ifdef ROS2
// We use the deprecated .h header instead of .hpp, because the latter is not available for foxy and humble.
// TODO Use .hpp header as soon as we do not support foxy and humble anymore
#ifdef ROS2_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#else
#include <cv_bridge/cv_bridge.h>
#endif
