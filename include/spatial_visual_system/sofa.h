#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_

#include <vector>
#include <mutex>

#include "spatial_visual_system/sofa_annotation.h"

#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <tmc_darknet_msgs/Detections.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <boost/shared_ptr.hpp>

#include <memory>

namespace svs {

struct Percept {
  Percept() = default;
  Percept(cv_bridge::CvImagePtr rgb, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
          tmc_darknet_msgs::Detections::ConstPtr yolo_detections) :
      rgb_{rgb}, cloud_{cloud}, yolo_detections_{yolo_detections} {
  };

  cv_bridge::CvImagePtr rgb_;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> cloud_;
  tmc_darknet_msgs::Detections::ConstPtr yolo_detections_;

};

class SofA {
public:
  SofA() {
    add_sofa_fields();
  }


  ~SofA() = default;
  // copy
  SofA(const SofA& sofa) = delete;
  SofA& operator=(const SofA&) = delete;
  // move
  SofA& operator=(SofA&& sofa) {
      if (this != &sofa) {
          annotations_ = std::move(sofa.annotations_);
          percept_ = std::move(sofa.percept_);
      }
      return *this;
  }
  SofA(SofA&& sofa) {
    *this = std::move(sofa);
  }

  std::mutex lock_;
  //std::vector<SofAAnnotation*> annotations_;
  nlohmann::json annotations_;
  Percept percept_;

private:
  void add_sofa_fields() {
    std::vector<std::string> fields = {
      "description", "associated_text", "pose_x", "pose_y", 
      "pose_z", "pose_w", "orient_r", "orient_p", "orient_yw",
      "loc_desc", "colour", "colour_low", "colour_high", "bounding_box", 
      "image_path", "features", "class_confidence", "fully_observed"};

    for (const auto& field : fields) {
      annotations_[field] = nullptr;
    }
  }
};

} // namespace svs

#endif
