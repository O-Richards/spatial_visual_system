#include "spatial_visual_system/yolo.h"
#include "spatial_visual_system/ros_utils.h"

#include <ros/ros.h>

#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace svs {
YoloGenerator::YoloGenerator(ros::NodeHandle& nh) :
    nh_{nh}
{
    // Setup aggregate objects
    read_params();
    yolo_service_ = nh_.serviceClient<unsw_vision_msgs::lookup>(yolo_service_name_);
    frame_rate_service_ = nh_.serviceClient<unsw_vision_msgs::setFrameRates>(set_frame_rate_service_name_);

    // Set the vision frame rate
    unsw_vision_msgs::setFrameRates frameRate;
    frameRate.request.person    = 0;
    frameRate.request.object    = 5;
    frameRate.request.furniture = 0;
    if (!frame_rate_service_.call(frameRate)) {
        ROS_ERROR("Failed to set vision frame rate!");
    }

    ROS_INFO("Finished setting up YoloGenerator");
}

void YoloGenerator::read_params() {
    get_ros_param(nh_, "yolo_results_topic", yolo_service_name_);
    get_ros_param(nh_, "set_frame_rate_service", set_frame_rate_service_name_);
    // get_ros_param(nh_, "yolo_debug", debug_, false);
}

void YoloGenerator::run(Scene& scene) {
    // Call the service
    unsw_vision_msgs::lookup lookup;
    if (!yolo_service_.call(lookup)) {
        ROS_ERROR("Failed calling lookup object on topic %s", yolo_service_name_.c_str());
        return;
    }

    std::vector<unsw_vision_msgs::Detection> lookup_detections = lookup.response.list;
    // Process the detections

    for (unsw_vision_msgs::Detection o : lookup_detections) {
        SofA* new_sofa = nullptr;
        // Create a new SofA
        {
        std::lock_guard<std::mutex> scene_lock{scene.lock_};
        new_sofa = &scene.addSofA();
        }
        // Lock the new sofa
        //std::lock_guard<std::mutex> sofa_lock(new_sofa->lock_);

        // Find class
        diagnostic_msgs::KeyValue class_key_val = *std::find_if(o.details.tags.begin(), o.details.tags.end(), 
                [&] (const diagnostic_msgs::KeyValue& key_val) {return key_val.key == o.details.KEY_CLASS;});

        std::string obj_class = class_key_val.value;
        rapidjson::Value obj_class_conf{0.0};
        
        rapidjson::Value obj_class_json;
        obj_class_json.SetString(obj_class.c_str(), obj_class.length(), new_sofa->annotations_.GetAllocator());
        new_sofa->annotations_.AddMember("class", obj_class_json, new_sofa->annotations_.GetAllocator());
        new_sofa->annotations_.AddMember("class_conf", obj_class_conf, new_sofa->annotations_.GetAllocator());
        
        if (debug_) {
            ROS_INFO("Dumping out SofA");
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            new_sofa->annotations_.Accept(writer);
            ROS_INFO_STREAM("New YOLO SofA: " << buffer.GetString());
        }
        
    }
}

} // namespace svs
