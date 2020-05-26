#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_

#include <vector>
#include <mutex>

#include "spatial_visual_system/sofa_annotation.h"

#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <boost/shared_ptr.hpp>

#include <memory>

namespace svs {

struct Percept {
  Percept() = default;
  Percept(cv_bridge::CvImagePtr rgb, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) :
    rgb_{rgb}, 
    cloud_{cloud} {};

  cv_bridge::CvImagePtr rgb_;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> cloud_;
};

class SofA {
public:
  SofA() = default;

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
};

} // namespace svs

#endif
