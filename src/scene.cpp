#include "spatial_visual_system/scene.h"

namespace svs {

void Scene::accept(SceneWriter &writer) {
    for (const auto& sofa : sofa_) {
        writer.write(sofa);
    }
}

} // namespace svs
