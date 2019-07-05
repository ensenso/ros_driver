#pragma once

#include <nodelet/nodelet.h>

#include <memory>

#include "ensenso_camera/stereo_camera.h"

namespace ensenso_camera
{
class Nodelet : public nodelet::Nodelet
{
private:
  std::unique_ptr<StereoCamera> camera;

public:
  void onInit() override;
  ~Nodelet() override;
};

}  // namespace ensenso_camera
