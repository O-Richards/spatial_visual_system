#ifndef SPATIAL_VISUAL_SYSTEM_SOFA_ANNOTATOR_H_
#define SPATIAL_VISUAL_SYSTEM_SOFA_ANNOTATOR_H_

#include "spatial_visual_system/sofa.h"
#include "spatial_visual_system/scene.h"

#include <vector>

#include <pcl/point_types.h>

#include <tf/transform_listener.h>

namespace svs {

class SofAAnnotator {
public:
  virtual ~SofAAnnotator() = default;
  virtual void read_params() {};
  virtual void run(const Scene& scene, std::vector<SofA>& sofa) = 0;
};

class ColourAnnotator : public SofAAnnotator {
public:
  virtual ~ColourAnnotator() = default;
  virtual void read_params() {};
  virtual void run(const Scene& scene, std::vector<SofA>& sofa);
private:
};

class PlaneAnnotator : public SofAAnnotator {
public:
  PlaneAnnotator(ros::NodeHandle& nh);
  virtual ~PlaneAnnotator() = default;
  virtual void run(const Scene& scene, std::vector<SofA>& sofa);
private:
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  const std::string working_frame_ = "base_footprint";
  ros::NodeHandlePtr nh_;
  ros::Publisher plane_pub_;
  tf::TransformListener tf_listener_;

  void removeGround(PointCloud::ConstPtr in_cloud, PointCloud::Ptr out_cloud);

};

class ShapeAnnotator : public SofAAnnotator {
public:
  ShapeAnnotator(ros::NodeHandle& nh);
  virtual ~ShapeAnnotator() = default;
  virtual void run(const Scene& scene, std::vector<SofA>& sofa);
};

class Size3DAnnotator : public SofAAnnotator {
public:
  Size3DAnnotator(ros::NodeHandle& nh) {};
  virtual ~Size3DAnnotator() = default;
  virtual void run(const Scene& scene, std::vector<SofA>& sofa);
  virtual void read_params();
private:
  const std::string camera_link_frame_ = "head_rgbd_sensor_link";
  const double small_to_med_thresh_ = 0.04; // m^2
  const double med_to_large_thresh = 0.08; // m^2
  const double large_to_v_large_thresh = 0.4; // m^2

  void groundArea(double area, std::string& semantic_label, double& confidence) const;
};

} // namespace svs
#endif
