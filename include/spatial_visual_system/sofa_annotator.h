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

} // namespace svs

#endif
