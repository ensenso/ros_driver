#pragma once

#include "nxLib.h"

struct VersionInfo{
  int majorV;
  int minorV;
  int buildV;
};

inline VersionInfo getCurrentNxLibVersion(){
  VersionInfo info;
  NxLibItem item;
  NxLibItem const& nxLibV = item[itmVersion][itmNxLib];

  info.majorV = nxLibV[itmMajor].asInt();
  info.minorV  = nxLibV[itmMinor].asInt();
  info.buildV = nxLibV[itmBuild].asInt();

  return info;
}