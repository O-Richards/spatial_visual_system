#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_COMMON_ANNOTATIONS_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_COMMON_ANNOTATIONS_H_

#include <geometry_msgs/PoseWithCovariance.h>

#include "spatial_visual_system/sofa_annotation.h"

namespace svs {
class LocationAnnotation : public SofAAnnotation {
 public:
  std::string to_json() const override { return ""; }

 private:
  geometry_msgs::PoseWithCovariance pose_;
};

}  // namespace svs

#endif
