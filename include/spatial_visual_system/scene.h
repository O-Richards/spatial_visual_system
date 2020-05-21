#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_

#include <vector>
#include <mutex>

#include <spatial_visual_system/sofa.h>

namespace svs {
class Scene {
public:
  SofA& addSofA() {auto ret = sofa_.emplace({}); return *ret;}
  std::mutex lock_;

private:
  std::vector<SofA> sofa_;
};
} // namespace svs

#endif
