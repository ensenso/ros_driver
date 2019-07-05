#pragma once

#include <nodelet/nodelet.h>

#include <memory>

#include "ensenso_camera/mono_camera.h"

namespace ensenso_camera
{
class NodeletMono : public nodelet::Nodelet
{
private:
  std::unique_ptr<MonoCamera> camera;

public:
  void onInit() override;
  ~NodeletMono() override;
};

}  // namespace ensenso_camera
