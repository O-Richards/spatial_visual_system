#ifndef SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_
#define SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_

#include "spatial_visual_system/sofa.h"

namespace svs {
// Using a visitor pattern
class SceneWriter {
public:
  void write(const SofA& sofa);
};
} // namespace svs

#endif
