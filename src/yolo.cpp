#include "spatial_visual_system/yolo.h"

#include <ros/ros.h>

#include <nlohmann/json.hpp>
#include <string>

#include "spatial_visual_system/ros_utils.h"

namespace svs {
YoloGenerator::YoloGenerator(ros::NodeHandle& nh) : nh_{nh} {
  // Setup aggregate objects
  read_params();
  yolo_service_ =
      nh_.serviceClient<unsw_vision_msgs::lookup>(yolo_service_name_);
  frame_rate_service_ = nh_.serviceClient<unsw_vision_msgs::setFrameRates>(
      set_frame_rate_service_name_);

  // Set the vision frame rate
  unsw_vision_msgs::setFrameRates frameRate;
  frameRate.request.person = 0;
  frameRate.request.object = 10;
  frameRate.request.furniture = 0;
  ROS_INFO("Waiting for service %s", set_frame_rate_service_name_.c_str());
  frame_rate_service_.waitForExistence();
  if (!frame_rate_service_.call(frameRate)) {
    ROS_ERROR("Failed to set vision frame rate!");
  }

  ROS_INFO("Finished setting up YoloGenerator");
}

void YoloGenerator::read_params() {
  get_ros_param(nh_, "yolo_results_topic", yolo_service_name_);
  get_ros_param(nh_, "set_frame_rate_service", set_frame_rate_service_name_);
  get_ros_param(nh_, "yolo_debug", debug_, false);
}

void YoloGenerator::run(Scene& scene) {
  // Call the Yolo service
  unsw_vision_msgs::lookup lookup;
  lookup.request.data_provided = true;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*scene.getPercept().cloud_, cloud_msg);
  lookup.request.cloud = cloud_msg;

  sensor_msgs::Image image_msg;
  scene.getPercept().rgb_->toImageMsg(image_msg);
  lookup.request.image = image_msg;

  lookup.request.detections = *scene.getPercept().yolo_detections_;

  if (!yolo_service_.call(lookup)) {
    ROS_ERROR("Failed calling lookup object on topic %s",
              yolo_service_name_.c_str());
    return;
  }

  std::vector<unsw_vision_msgs::Detection> lookup_detections =
      lookup.response.list;
  // Process the detections

  for (unsw_vision_msgs::Detection o : lookup_detections) {
    SofA* new_sofa = nullptr;
    // Create a new SofA
    {
      std::lock_guard<std::mutex> scene_lock{scene.lock_};
      new_sofa = &scene.addSofA();
    }
    // Lock the new sofa
    std::lock_guard<std::mutex> sofa_lock(new_sofa->lock_);

    // Find class
    diagnostic_msgs::KeyValue class_key_val =
        *std::find_if(o.details.tags.begin(), o.details.tags.end(),
                      [&](const diagnostic_msgs::KeyValue& key_val) {
                        return key_val.key == o.details.KEY_CLASS;
                      });

    diagnostic_msgs::KeyValue class_conf_key_val =
        *std::find_if(o.details.tags.begin(), o.details.tags.end(),
                      [&](const diagnostic_msgs::KeyValue& key_val) {
                        return key_val.key == o.details.KEY_CLASS_CONF;
                      });

    std::string obj_class = class_key_val.value;
    double obj_class_conf = std::stod(class_conf_key_val.value);

    // Write out class
    new_sofa->annotations_["object_category"] = obj_class;
    new_sofa->annotations_["class_confidence"] = obj_class_conf;

    // Write out point
    new_sofa->annotations_["frame_id"] = o.details.frame_id;
    new_sofa->annotations_["pose_x"] = o.details.position.x;
    new_sofa->annotations_["pose_y"] = o.details.position.y;
    new_sofa->annotations_["pose_z"] = o.details.position.z;

    if (debug_) {
      ROS_INFO_STREAM(
          "YoloGenerator made new sofa: " << new_sofa->annotations_);
    }
  }
}

}  // namespace svs
