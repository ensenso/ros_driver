/*
 * With both ROS1 nodelets and ROS2 components it is possible to start two camera nodes within the same process. When it
 * comes to closing the NxLib via nxLibFinalize, only the last open camera in that process should perform the closing.
 * This singleton class performs nxLibInitialize on construction and nxLibFinalize on destruction accordingly and makes
 * sure that the NxLib is implicitly closed after all camera nodes have been closed.
 */
class NxLibInitializeFinalize
{
public:
  static NxLibInitializeFinalize& instance();

  NxLibInitializeFinalize(const NxLibInitializeFinalize&) = delete;
  NxLibInitializeFinalize& operator=(const NxLibInitializeFinalize&) = delete;

private:
  NxLibInitializeFinalize();
  ~NxLibInitializeFinalize();

private:
  bool initialized = false;
};
