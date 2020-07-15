#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_GENERATOR_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_GENERATOR_H_

#include "spatial_visual_system/scene.h"
#include "spatial_visual_system/sofa.h"

namespace svs {
class SofAGenerator {
public:
  virtual ~SofAGenerator() = default;
  virtual void read_params() {};
  virtual void run(Scene& scene) = 0;
};

} // namespace svs

#endif
