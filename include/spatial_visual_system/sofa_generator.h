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

class SofAAnnotator {
public:
  virtual ~SofAAnnotator() = default;
  virtual void read_params() {};
  virtual void run(const Scene& scene, SofA& sofa) = 0;
};
} // namespace svs

#endif
