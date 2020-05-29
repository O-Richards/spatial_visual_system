#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_ROS_UTILS_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_ROS_UTILS_H_

#include <ros/node_handle.h>

namespace svs {

template <typename T>
bool get_ros_param(const ros::NodeHandle& nh, std::string param_name, T& result,
                   const T& default_value) {
  if (nh.getParam(param_name, result) == false) {
    ROS_WARN("Parameter %s does not exist. Defaulting to %s",
             param_name.c_str(), std::to_string(default_value).c_str());
    result = default_value;
    return false;
  }
  return true;
}

template <typename T>
bool get_ros_param(const ros::NodeHandle& nh, std::string param_name,
                   T& result) {
  if (nh.getParam(param_name, result) == false) {
    ROS_ERROR("Parameter %s does not exist.", param_name.c_str());
    return false;
  }
  return true;
}

}  // namespace svs
#endif
