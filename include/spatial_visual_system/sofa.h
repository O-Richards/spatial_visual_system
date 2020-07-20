#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SOFA_H_

#include <vector>
#include <mutex>

#include "spatial_visual_system/sofa_annotation.h"

#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include <tmc_darknet_msgs/Detections.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

#include <memory>

namespace svs {

struct Percept {
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  using Normal = pcl::Normal;
  using NormalCloud = pcl::PointCloud<Normal>;

  Percept() = default;
  Percept(const Percept&) = default;
  Percept(Percept&& s) = default;
  Percept& operator=(Percept&&) = default;
  Percept& operator=(const Percept&) = default;
  ~Percept() = default;

  Percept(cv_bridge::CvImagePtr rgb, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) :
      rgb_{rgb}, cloud_{cloud}, cloud_normals_{nullptr} {
          if (cloud_ != nullptr) {
              // Calculate cloud normal
              pcl::NormalEstimation<Point, Normal> normal_calc{};
              normal_calc.setInputCloud(cloud_);
              // Create an empty kdtree representation, and pass it to the normal estimation object.
              // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
              pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
              normal_calc.setSearchMethod(tree);
              // Use all neighbors in a sphere of radius 3cm
              normal_calc.setRadiusSearch (0.03);
              auto cloud_norm = boost::make_shared<NormalCloud>();
              normal_calc.compute(*cloud_norm);
              cloud_normals_ = cloud_norm;
          }
  };

  cv_bridge::CvImageConstPtr rgb_;
  PointCloud::ConstPtr cloud_;
  NormalCloud::ConstPtr cloud_normals_;
};

class SofA {
public:
  SofA() :
    id_{next_sofa_no_++} {
    add_sofa_fields();
  }
  SofA(const SofA& sofa) = delete;

  ~SofA() = default;
  // copy
  SofA& operator=(const SofA&) = delete;
  // move
  SofA& operator=(SofA&& sofa) {
      if (this != &sofa) {
          annotations_ = std::move(sofa.annotations_);
          percept_ = std::move(sofa.percept_);
          cloud_index_mask_ = std::move(sofa.cloud_index_mask_);
          id_ = std::move(sofa.id_);
      }
      return *this;
  }
  SofA(SofA&& sofa) {
    *this = std::move(sofa);
  }

  unsigned int getId() const {
    return id_;
  }

  std::mutex lock_;
  //std::vector<SofAAnnotation*> annotations_;
  nlohmann::json annotations_;
  Percept percept_;
  std::vector<int> cloud_index_mask_;
  boost::optional<cv::Rect> bbox_;

private:
  static unsigned int next_sofa_no_;
  unsigned int id_;
  void add_sofa_fields() {
    std::vector<std::string> fields = {
        "description",      "associated_text", "pose_x",     "pose_y",
        "pose_z",           "pose_w",          "orient_r",   "orient_p",
        "orient_yw",        "loc_desc",        "colour",     "colour_low",
        "colour_high",      "bounding_box",    "image_path", "features",
        "fully_observed", "frame_id", "labels"};

    for (const auto& field : fields) {
      annotations_[field] = nullptr;
    }
    // TODO: @oliri detect fully_observed properly
    annotations_["fully_observed"] = false;
  }
};


} // namespace svs

#endif
