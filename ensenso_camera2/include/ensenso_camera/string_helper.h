#pragma once

#include <stdlib.h>

inline bool startswith(std::string const& lhs, std::string const& rhs)
{
  return lhs.rfind(rhs, 0) == 0;
}

inline std::string expandPath(std::string const& path_)
{
  std::string path(path_);
  if (startswith(path, "~/"))
  {
    path = getenv("HOME") + std::string("/") + path.substr(2, std::string::npos);
  }
  return path;
}
