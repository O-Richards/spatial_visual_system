#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_

#include <vector>
#include <mutex>

#include "spatial_visual_system/sofa_annotation.h"

#include <nlohmann/json.hpp>

namespace svs {
class SofA {
public:
  SofA() = default;

  ~SofA() = default;
  // copy
  SofA(const SofA& sofa) = delete;
  SofA& operator=(const SofA&) = delete;
  // move
  SofA& operator=(SofA&& sofa) {
      if (this != &sofa) {
          annotations_ = std::move(sofa.annotations_);
      }
      return *this;
  }
  SofA(SofA&& sofa) {
    *this = std::move(sofa);
  }

  std::mutex lock_;
  //std::vector<SofAAnnotation*> annotations_;
  nlohmann::json annotations_;
};

} // namespace svs

#endif
