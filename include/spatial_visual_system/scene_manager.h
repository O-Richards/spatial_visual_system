#ifndef SPATIAL_VISUAL_SYSTEM_SCENE_MANAGER_H_
#define SPATIAL_VISUAL_SYSTEM_SCENE_MANAGER_H_

#include "spatial_visual_system/scene.h"
#include "spatial_visual_system/yolo.h"
#include "spatial_visual_system/scene_writer.h"
#include "spatial_visual_system/sofa_annotator.h"

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

#include <unsw_vision_msgs/DetectionList.h>

#include <memory>

namespace svs {

class SceneManager {
private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, unsw_vision_msgs::DetectionList>;
  const int queue_size_ = 10; 

  ros::NodeHandle& nh_;
  svs::Scene scene_;
  svs::YoloGenerator yolo_generator_;
  svs::ColourAnnotator colour_annotator_;
  svs::PlaneAnnotator plane_annotator_;
  svs::ShapeAnnotator shape_annotator_;
  svs::Size3DAnnotator size_annotator_;
  svs::SiftRecognitionAnnotator sift_annotator_;

  svs::SceneWriter scene_writer_;

  double svs_freq_ = 40;
  //ros::Timer tick_timer_;


  // For subscribing to sensor data
  std::unique_ptr<image_transport::SubscriberFilter> sub_image_colour_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;
  std::unique_ptr<message_filters::Subscriber<unsw_vision_msgs::DetectionList>> sub_detections_;

  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sub_sync_;

  // last sensor data frame
  sensor_msgs::Image::ConstPtr last_image_colour_ = nullptr;
  sensor_msgs::PointCloud2::ConstPtr last_cloud_ = nullptr;
  unsw_vision_msgs::DetectionList::ConstPtr last_detections_ = nullptr;

  std::mutex data_mutex_;

public:
  SceneManager(ros::NodeHandle& nh);

  void sync_cb(const sensor_msgs::Image::ConstPtr image_colour, const sensor_msgs::PointCloud2ConstPtr cloud, 
          const unsw_vision_msgs::DetectionListConstPtr detections) {
      {
          std::lock_guard<std::mutex> lock{data_mutex_};
          last_image_colour_ = image_colour;
          last_cloud_ = cloud;
          last_detections_ = detections;
      }
      tick();
  }

  void tick();
  void tick(const ros::TimerEvent& event);
};


} // namespace svs

#endif
