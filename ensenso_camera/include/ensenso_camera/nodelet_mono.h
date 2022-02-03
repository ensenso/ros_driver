#pragma once

#include "ensenso_camera/mono_camera.h"

#include <nodelet/nodelet.h>

#include <memory>

namespace ensenso_camera
{
class MonoCameraNodelet : public nodelet::Nodelet
{
public:
  MonoCameraNodelet();
  ~MonoCameraNodelet();

  void onInit() override;

private:
  std::string cameraType;
  std::unique_ptr<MonoCamera> camera;
};
}  // namespace ensenso_camera
