#ifndef SPATIAL_VISUAL_SYSTEM_SCENE_MANAGER_H_
#define SPATIAL_VISUAL_SYSTEM_SCENE_MANAGER_H_

#include "spatial_visual_system/scene.h"
#include "spatial_visual_system/yolo.h"
#include "spatial_visual_system/scene_writer.h"

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/subscriber.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <tmc_darknet_msgs/Detections.h>

#include <memory>

namespace svs {

class SceneManager {
private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>;
  const int queue_size_ = 10; 

  ros::NodeHandle& nh_;
  svs::Scene scene_;
  svs::YoloGenerator yolo_generator_;
  svs::SceneWriter scene_writer_;

  double svs_freq_ = 10;
  ros::Rate rate_controller_;


  // For subscribing to sensor data
  std::unique_ptr<image_transport::SubscriberFilter> sub_image_colour_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;

  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sub_sync_;

  // last sensor data frame
  sensor_msgs::Image::ConstPtr last_image_colour_ = nullptr;
  sensor_msgs::PointCloud2::ConstPtr last_cloud_ = nullptr;

  std::mutex data_mutex_;

public:
  SceneManager(ros::NodeHandle& nh):
    nh_{nh}, yolo_generator_{nh_}, scene_writer_{nh_}, rate_controller_{svs_freq_}
  {
    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
    image_transport::TransportHints hints("raw");
  
    sub_image_colour_ = 
        std::make_unique<image_transport::SubscriberFilter>(it, "image", queue_size_, hints);
  
    sub_cloud_ = 
        std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, "points", queue_size_);

    sub_sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_), *sub_image_colour_, *sub_cloud_);
    sub_sync_->registerCallback(boost::bind(&SceneManager::sync_cb, this, _1, _2));
  };

  void sync_cb(const sensor_msgs::Image::ConstPtr image_colour, const sensor_msgs::PointCloud2ConstPtr cloud) {
      std::lock_guard<std::mutex> lock{data_mutex_};
      last_image_colour_ = image_colour;
      last_cloud_ = cloud;
  }

  void tick();
};


} // namespace svs

#endif
