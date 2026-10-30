#include "ensenso_camera/nxlib_initialize_finalize.h"

#include <iostream>

#include "nxLib.h"

NxLibInitializeFinalize& NxLibInitializeFinalize::instance()
{
  static NxLibInitializeFinalize nxLibInitializeFinalize;
  return nxLibInitializeFinalize;
}

NxLibInitializeFinalize::NxLibInitializeFinalize()
{
  nxLibInitialize(/*waitForInitialCameraRefresh=*/true);
  initialized = true;
}

NxLibInitializeFinalize::~NxLibInitializeFinalize()
{
  if (initialized)
  {
    try
    {
      nxLibFinalize();
    }
    catch (NxLibException& e)
    {
    }
  }
}
