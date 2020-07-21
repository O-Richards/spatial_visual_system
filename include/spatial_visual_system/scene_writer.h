#ifndef SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_
#define SPATIAL_VISUAL_SYSTEM_SCENE_WRITER_H_

#include "spatial_visual_system/sofa.h"

#include <nlohmann/json.hpp>

#include <ros/ros.h>

#include <world_model_store_msgs/Insert.h>

#include <spatial_visual_system/Detection.h>

namespace svs {
#define USE_SERVICE 0
// Using a visitor pattern
class SceneWriter {
public:
  SceneWriter(ros::NodeHandle& nh) :
#if USE_SERVICE
      new_scene_serv_{nh.serviceClient<world_model_store_msgs::Insert>("/world_model/insert_scene")},
      new_scene_obj_serv_{nh.serviceClient<world_model_store_msgs::Insert>("/world_model/insert_scene_object")},
      new_robot_state_serv_{nh.serviceClient<world_model_store_msgs::Insert>("/world_model/insert_robot_state")},
#endif
      curr_scene_id_{0}
    {
#if USE_SERVICE
        new_scene_serv_.waitForExistence();
        new_scene_obj_serv_.waitForExistence();
        new_robot_state_serv_.waitForExistence();
#endif

        detection_pub_ = nh.advertise<spatial_visual_system::Detection>("detections", 100);
        ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "Setup SceneWriter");
    }

  void new_scene();
  void write(SofA& sofa);

private:
  bool debug_{true};

  nlohmann::json partial_scene_;
#if USE_SERVICE
  ros::ServiceClient new_scene_serv_;
  ros::ServiceClient new_scene_obj_serv_;
  ros::ServiceClient new_robot_state_serv_;
#endif

  ros::Publisher detection_pub_;

  int curr_scene_id_;

  int new_robot_state();
};
} // namespace svs

#endif
