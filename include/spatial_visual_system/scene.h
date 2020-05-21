#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_

#include <vector>
#include <mutex>

#include "spatial_visual_system/sofa.h"
#include "spatial_visual_system/scene_writer.h"

namespace svs {
class Scene {
public:
  SofA& addSofA() {sofa_.push_back(SofA{}); return sofa_.back();}
  void accept(SceneWriter& writer);
  std::mutex lock_;

private:
  std::vector<SofA> sofa_;
};
} // namespace svs

#endif
