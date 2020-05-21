#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_YOLO_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_YOLO_H_

#include "spatial_visual_system/sofa_generator.h"

#include <unsw_vision_msgs/lookup.h>
#include <unsw_vision_msgs/Detection.h>
#include <unsw_vision_msgs/setFrameRates.h>

#include <ros/node_handle.h>

namespace svs {
class YoloGenerator : public SofAGenerator {
public:
  YoloGenerator(ros::NodeHandle& nh);
  virtual ~YoloGenerator() = default;

  virtual void read_params();
  virtual void run(Scene& scene);
private:
  std::string yolo_service_name_;
  std::string set_frame_rate_service_name_;
  ros::NodeHandle& nh_;
  ros::ServiceClient yolo_service_;
  ros::ServiceClient frame_rate_service_;
  bool debug_ = true;
};
} // namespace svs

#endif
