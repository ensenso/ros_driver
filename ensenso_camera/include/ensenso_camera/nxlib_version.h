#pragma once

#include "nxLib.h"

#include <string>

struct NxLibVersion
{
  int major;
  int minor;
  int build;

  /**
   * Return the current NxLib version information.
   */
  void fillFromNxLib()
  {
    NxLibItem item;
    NxLibItem const& nxLibV = item[itmVersion][itmNxLib];

    major = nxLibV[itmMajor].asInt();
    minor = nxLibV[itmMinor].asInt();
    build = nxLibV[itmBuild].asInt();
  }

  /**
   * Check whether the given required version is met.
   */
  bool meetsMinimumRequirement(int majorRequired, int minorRequired) const
  {
    return (major > majorRequired) || (major == majorRequired && minor >= minorRequired);
  }

  /**
   * Return the version number as string with format <major.minor.build>.
   */
  std::string toString() const
  {
    return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(build);
  }
};
