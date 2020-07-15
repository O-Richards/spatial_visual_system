#include <pluginlib/class_list_macros.h>

#include "spatial_visual_system/svs_nodelet.h"
#include "spatial_visual_system/scene_manager.h"

PLUGINLIB_EXPORT_CLASS(svs::SvsNodelet, nodelet::Nodelet)

namespace svs {
void SvsNodelet::onInit() {
    NODELET_INFO("Beep boop... starting up spatial_visual_system");

    auto& nh = getPrivateNodeHandle();
    svs::SceneManager scene_manager{nh};

    ROS_INFO("Ticking scene_manager. No explicit spins!");
    ros::spin();

    ROS_INFO("spatial_visual_system shutting down!");
}
}
