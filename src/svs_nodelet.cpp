#include <pluginlib/class_list_macros.h>

#include "spatial_visual_system/svs_nodelet.h"
#include "spatial_visual_system/scene_manager.h"

#include <memory>

PLUGINLIB_EXPORT_CLASS(svs::SvsNodelet, nodelet::Nodelet)

namespace svs {
void SvsNodelet::onInit() {
    NODELET_INFO("Beep boop... starting up spatial_visual_system");

    auto& nh = getPrivateNodeHandle();
    scene_manager_ = std::make_unique<svs::SceneManager>(nh);

    ROS_INFO("Finished setting up Spatial Visual System");

}
}
