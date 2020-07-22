#include "spatial_visual_system/yolo.h"
#include "spatial_visual_system/ros_utils.h"
#include "spatial_visual_system/3d_helpers.h"

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <nlohmann/json.hpp>

namespace svs {
YoloGenerator::YoloGenerator(ros::NodeHandle& nh) :
    nh_{nh}
{
    // Setup aggregate objects
    read_params();
    yolo_service_ = nh_.serviceClient<unsw_vision_msgs::lookup>(yolo_service_name_);
    frame_rate_service_ = nh_.serviceClient<unsw_vision_msgs::setFrameRates>(set_frame_rate_service_name_);

    // Set the vision frame rate
    /*
    unsw_vision_msgs::setFrameRates frameRate;
    frameRate.request.person    = 0;
    frameRate.request.object    = 10;
    frameRate.request.furniture = 0;
    ROS_INFO("Waiting for service %s", set_frame_rate_service_name_.c_str());
    frame_rate_service_.waitForExistence();
    if (!frame_rate_service_.call(frameRate)) {
        ROS_ERROR("Failed to set vision frame rate!");
    }
    */

    ROS_INFO("Finished setting up YoloGenerator");
}

void YoloGenerator::read_params() {
    get_ros_param(nh_, "yolo_results_topic", yolo_service_name_);
    get_ros_param(nh_, "set_frame_rate_service", set_frame_rate_service_name_);
    get_ros_param(nh_, "yolo_debug", debug_, false);
}

void YoloGenerator::run(Scene& scene) {
    // Call the Yolo service
    /*
    unsw_vision_msgs::lookup lookup;
    lookup.request.data_provided = true;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*scene.getPercept().cloud_, cloud_msg);
    lookup.request.cloud = cloud_msg;

    sensor_msgs::Image image_msg;
    scene.getPercept().rgb_->toImageMsg(image_msg);
    lookup.request.image = image_msg;

    if (!yolo_service_.call(lookup)) {
        ROS_ERROR("Failed calling lookup object on topic %s", yolo_service_name_.c_str());
        return;
    }
    */

    std::vector<unsw_vision_msgs::Detection> lookup_detections = scene.copyDetectionsWLock().list;
    // Process the detections

    for (unsw_vision_msgs::Detection o : lookup_detections) {
        SofA *new_sofa = nullptr;
        // Create a new SofA
        {
            std::lock_guard<std::mutex> scene_lock{scene.lock_};
            new_sofa = &scene.addSofA();
        }
        // Lock the new sofa
        std::lock_guard<std::mutex> sofa_lock(new_sofa->lock_);

        ROS_INFO_STREAM("bbox:" << o.details.bbox);
        // Add in the Percept
        cv_bridge::CvImage img{};
        img.header = scene.getPercept().rgb_->header;
        img.encoding = scene.getPercept().rgb_->encoding;
        auto boundry = cv::Rect{{static_cast<int>(o.details.bbox.x), static_cast<int>(o.details.bbox.y)},
                cv::Size{static_cast<int>(o.details.bbox.width), static_cast<int>(o.details.bbox.height)}};
        cv::Mat roi = scene.getPercept().rgb_->image(boundry);
        img.image = roi;
        new_sofa->percept_.rgb_ = boost::make_shared<const cv_bridge::CvImage>(img);

        // Find cloud indicies
        int img_cols = scene.getPercept().rgb_->image.cols;
        std::vector<int> cloud_indexs{};
        svs::extractCloudFromBbox(scene.getPercept().cloud_, img_cols, boundry, cloud_indexs);
        new_sofa->cloud_index_mask_ = cloud_indexs;

        new_sofa->detection_ = o;

        // Find class
        diagnostic_msgs::KeyValue class_key_val = *std::find_if(o.details.tags.begin(), o.details.tags.end(), 
                [&] (const diagnostic_msgs::KeyValue& key_val) {return key_val.key == o.details.KEY_CLASS;});

        diagnostic_msgs::KeyValue class_conf_key_val = *std::find_if(o.details.tags.begin(), o.details.tags.end(), 
                [&] (const diagnostic_msgs::KeyValue& key_val) {return key_val.key == o.details.KEY_CLASS_CONF;});

        std::string obj_class = class_key_val.value;
        double obj_class_conf = std::stod(class_conf_key_val.value);

        new_sofa->annotations_["object_category"] = obj_class;
        new_sofa->annotations_["class_confidence"] = obj_class_conf;

        // Write out class
        if (new_sofa->annotations_["labels"] == nullptr) {
            new_sofa->annotations_["labels"] = std::vector<std::tuple<std::string, double>>{};
        }
        new_sofa->annotations_["labels"].push_back({obj_class, obj_class_conf});

        // Write out point
        new_sofa->annotations_["frame_id"] = o.details.frame_id;
        new_sofa->annotations_["pose_x"] = o.details.position.x;

        new_sofa->annotations_["pose_y"] = o.details.position.y;
        new_sofa->annotations_["pose_z"] = o.details.position.z;

        // Write out image bounding box (opencv style point at top left + width & height)
        new_sofa->bbox_ = boundry;
        
        if (debug_) {
            ROS_INFO_STREAM("YoloGenerator made new sofa: " << new_sofa->annotations_);
        }
    }
}

} // namespace svs
