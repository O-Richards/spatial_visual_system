#ifndef SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_
#define SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_

#include "spatial_visual_system/sofa.h"

#include <nlohmann/json.hpp>

namespace svs {
// Using a visitor pattern
class SceneWriter {
public:
  void reset();
  void visit(const SofA& sofa);
  void writeOut();
private:
  nlohmann::json partial_scene_;
};
} // namespace svs

#endif
