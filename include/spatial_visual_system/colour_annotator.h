#ifndef SPATIAL_VISUAL_SYSTEM_COLOUR_ANNOTATOR_H_
#define SPATIAL_VISUAL_SYSTEM_COLOUR_ANNOTATOR_H_

#include "sofa_generator.h"

namespace svs {

class ColourAnnotator : public SofAAnnotator {
public:
  virtual ~ColourAnnotator() = default;
  virtual void read_params() {};
  virtual void run(const Scene& scene, SofA& sofa);
private:
};

} // namespace svs

#endif
