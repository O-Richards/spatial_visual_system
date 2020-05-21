#ifndef SPATIAL_VISUAL_SYSTEM_SOFA_ANNOTATION_H_
#define SPATIAL_VISUAL_SYSTEM_SOFA_ANNOTATION_H_

#include <string>


namespace svs {
class SofAAnnotation {
    virtual std::string to_json() const;
};
} // namespace svs

#endif
