#include "spatial_visual_system/scene_writer.h"

namespace svs {
void SceneWriter::visit(const SofA &sofa) {
    partial_scene_.push_back(sofa.annotations_);
}

void SceneWriter::writeOut() {
}

void SceneWriter::reset() {
    partial_scene_ = nlohmann::json();
}

} // namespace svs
