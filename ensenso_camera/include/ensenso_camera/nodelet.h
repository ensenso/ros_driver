#pragma once

#include <nodelet/nodelet.h>

#include <memory>

#include "ensenso_camera/camera.h"

namespace ensenso_camera
{

class Nodelet : public nodelet::Nodelet
{
private:
  std::unique_ptr<Camera> camera;

public:
  void onInit() override;
  virtual ~Nodelet();
};

}  // namespace ensenso_camera
