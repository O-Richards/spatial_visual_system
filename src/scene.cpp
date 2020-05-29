#include "spatial_visual_system/scene.h"

#include "spatial_visual_system/scene_writer.h"

namespace svs {

void Scene::accept(SceneWriter& writer) {
  writer.new_scene();
  for (auto& sofa : sofa_) {
    writer.write(sofa);
  }
}

}  // namespace svs
