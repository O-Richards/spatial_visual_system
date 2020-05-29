#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_

#include <mutex>
#include <vector>

#include "spatial_visual_system/scene_writer.h"
#include "spatial_visual_system/sofa.h"

namespace svs {
class Scene {
 public:
  SofA& addSofA() {
    sofa_.push_back(SofA{});
    return sofa_.back();
  }
  void accept(SceneWriter& writer);
  void reset() { sofa_.clear(); };
  void setPercept(const Percept& percept) { percept_ = percept; }
  const Percept& getPercept() const { return percept_; };

  std::mutex lock_;

 private:
  std::vector<SofA> sofa_;
  Percept percept_;
};
}  // namespace svs

#endif
