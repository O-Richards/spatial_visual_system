#ifndef SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_
#define SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_

#include "spatial_visual_system/sofa.h"

#include <nlohmann/json.hpp>

#include <ros/ros.h>

#include <world_model_store_msgs/Insert.h>

namespace svs {
// Using a visitor pattern
class SceneWriter {
public:
  SceneWriter(ros::NodeHandle& nh) :
      new_scene_serv_{nh.serviceClient<world_model_store_msgs::Insert>("/world_model/insert_scene")},
      new_scene_obj_serv_{nh.serviceClient<world_model_store_msgs::Insert>("/world_model/insert_scene_object")},
      new_robot_state_serv_{nh.serviceClient<world_model_store_msgs::Insert>("/world_model/insert_robot_state")}
    {
        new_scene_serv_.waitForExistence();
        new_scene_obj_serv_.waitForExistence();
        new_robot_state_serv_.waitForExistence();
    }

  void new_scene();
  void write(SofA& sofa);

private:
  bool debug_{true};

  nlohmann::json partial_scene_;
  ros::ServiceClient new_scene_serv_;
  ros::ServiceClient new_scene_obj_serv_;
  ros::ServiceClient new_robot_state_serv_;
  int curr_scene_id_;

  int new_robot_state();
};
} // namespace svs

#endif
