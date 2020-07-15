#ifndef SPATIAL_VISUAL_SYSTEM_SOFA_ANNOTATOR_H_
#define SPATIAL_VISUAL_SYSTEM_SOFA_ANNOTATOR_H_

#include "spatial_visual_system/sofa.h"
#include "spatial_visual_system/scene.h"

#include <vector>

namespace svs {

class SofAAnnotator {
public:
  virtual ~SofAAnnotator() = default;
  virtual void read_params() {};
  virtual void run(const Scene& scene, std::vector<SofA>& sofa) = 0;
};

class ColourAnnotator : public SofAAnnotator {
public:
  virtual ~ColourAnnotator() = default;
  virtual void read_params() {};
  virtual void run(const Scene& scene, std::vector<SofA>& sofa);
private:
};

class PlaneAnnotator : public SofAAnnotator {
public:
  virtual ~PlaneAnnotator() = default;
  virtual void run(const Scene& scene, std::vector<SofA>& sofa);

};

} // namespace svs

#endif
