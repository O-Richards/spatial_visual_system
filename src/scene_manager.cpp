#include "spatial_visual_system/scene_manager.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace svs {

void SceneManager::tick() {
    if (last_cloud_ == nullptr || last_image_colour_ == nullptr || last_yolo_detections_ == nullptr) return;
    std::cout << "tick\n";

    /* Create local copy of data */
    cv_bridge::CvImagePtr image_cv;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud{new pcl::PointCloud<pcl::PointXYZ>};
    // local image as imageCV
    try {
      image_cv = cv_bridge::toCvCopy(last_image_colour_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Percept(): cv_bridge exception: %s", e.what());
    }

    pcl::fromROSMsg (*last_cloud_, *pcl_cloud);
    Percept curr_percept{image_cv, pcl_cloud, last_yolo_detections_};

    // Create a new scene
    {
        std::lock_guard<std::mutex> scene_lock{scene_.lock_};
        scene_.reset();
        scene_.setPercept(curr_percept);
    }
    yolo_generator_.run(scene_); rate_controller_.sleep();
}

} // namespace svs
