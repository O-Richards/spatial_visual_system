#ifndef SVS_NODELET_H_
#define SVS_NODELET_H_

#include "spatial_visual_system/scene_manager.h"

#include <nodelet/nodelet.h>

namespace svs {
class SvsNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit();
  private:
    std::unique_ptr<svs::SceneManager> scene_manager_ = nullptr;
};
} // namespace svs
#endif
