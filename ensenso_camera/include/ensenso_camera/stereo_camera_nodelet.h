#pragma once

#include "ensenso_camera/stereo_camera.h"

#include <nodelet/nodelet.h>

#include <memory>

namespace ensenso_camera
{
class StereoCameraNodelet : public nodelet::Nodelet
{
public:
  StereoCameraNodelet();
  ~StereoCameraNodelet();

  void onInit() override;

private:
  std::string cameraType;
  std::unique_ptr<StereoCamera> camera;
};
}  // namespace ensenso_camera
