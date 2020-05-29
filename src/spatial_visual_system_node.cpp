#include <ros/node_handle.h>
#include <ros/ros.h>

#include "spatial_visual_system/scene_manager.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "spatial_visual_system");
  ros::NodeHandle nh{"~"};
  ROS_INFO("Beep boop... starting up spatial_visual_system");

  svs::SceneManager scene_manager{nh};

  ROS_INFO("Ticking scene_manager. No explicit spins!");
  while (ros::ok()) {
    scene_manager.tick();
    ros::spinOnce();
  }

  ROS_INFO("spatial_visual_system shutting down!");
}
