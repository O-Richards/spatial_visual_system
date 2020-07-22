#include "spatial_visual_system/sofa_annotator.h"

#include <unsw_vision_msgs/RecogniseObjects.h>

namespace svs {

#define DEBUG 1

SiftRecognitionAnnotator::SiftRecognitionAnnotator(ros::NodeHandle& nh) {
    recognition_client_ = nh.serviceClient<unsw_vision_msgs::RecogniseObjects>(recognition_topic_);
}

void SiftRecognitionAnnotator::run(const Scene& scene, std::vector<SofA>& sofa) {
    for (auto& object : sofa) {
        std::lock_guard<decltype(object.lock_)> lock{object.lock_};

        // Setup service call
        unsw_vision_msgs::RecogniseObjects recognise{};
        unsw_vision_msgs::Detection detection = object.detection_;
        detection.image = *scene.getPercept().rgb_->toImageMsg();
        recognise.request.list.push_back(detection);
#if DEBUG
        ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Calling recognition_client");
#endif

        if (recognition_client_.call(recognise)) {
#if DEBUG
            ROS_INFO_STREAM("\t Got response from recognition_client");
#endif
            auto recog_result = recognise.response.list[0].details.tags;
            // Find the sub-cat tag
            auto sub_key_val = std::find_if(recog_result.begin(), recog_result.end(), 
                    [](const auto& key_val){
                    return key_val.key == unsw_vision_msgs::DetectionDetails::KEY_SUBCLASS;});
            if (sub_key_val != recog_result.end()) {
                // We got a class to match with!
                auto sub_cat = sub_key_val->value;
                
                // TODO make recognition send this through
                double sub_cat_conf = NAN;
                auto sub_conf = std::find_if(recog_result.begin(), recog_result.end(), 
                        [](const auto& key_val){
                        return key_val.key == unsw_vision_msgs::DetectionDetails::KEY_SUBCLASS_CONF;});
                if (sub_conf == recog_result.end()) {
                    ROS_WARN_STREAM(__PRETTY_FUNCTION__ << 
                            " Recognition result contains a subclass but not a confidence");
                } else {
                    std::istringstream is{sub_conf->value};
                    is >> sub_cat_conf;
                }
                // Write out class
                object.annotations_["object_subcategory"] = sub_cat;
                if (object.annotations_["labels"] == nullptr) {
                    object.annotations_["labels"] = std::vector<std::tuple<std::string, double>>{};
                }
                object.annotations_["labels"].push_back({sub_cat, sub_cat_conf});
            } // else nothing recognised
        } else {
            ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Error calling recognition service");
        }
    }
}


} // namespace svs
