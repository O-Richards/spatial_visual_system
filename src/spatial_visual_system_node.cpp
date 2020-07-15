#include <ros/ros.h>
#include <ros/node_handle.h>

#include "spatial_visual_system/scene_manager.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "spatial_visual_system");
    ros::NodeHandle nh{"~"};
    ROS_INFO("Beep boop... starting up spatial_visual_system");

    svs::SceneManager scene_manager{nh};
    
    ros::spin();

    ROS_INFO("spatial_visual_system shutting down!");
}
