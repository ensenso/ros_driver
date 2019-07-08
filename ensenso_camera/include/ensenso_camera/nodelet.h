#pragma once

#include "ensenso_camera/stereo_camera.h"

#include <nodelet/nodelet.h>

#include <memory>

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
