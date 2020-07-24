#include "spatial_visual_system/scene_manager.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace svs {

SceneManager::SceneManager(ros::NodeHandle& nh):
    nh_{nh}, yolo_generator_{nh_}, plane_annotator_{nh_}, shape_annotator_{nh_}, size_annotator_{nh_}, scene_writer_{nh_}
{
    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
    image_transport::TransportHints hints("raw");

    sub_image_colour_ = 
        std::make_unique<image_transport::SubscriberFilter>(it, "image", queue_size_, hints);

    sub_cloud_ = 
        std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, "points", queue_size_);

    sub_sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_), *sub_image_colour_, *sub_cloud_);
    sub_sync_->registerCallback(boost::bind(&SceneManager::sync_cb, this, _1, _2));

    // setup timer
    tick_timer_ = nh_.createTimer(ros::Duration(1/svs_freq_), boost::bind(&SceneManager::tick, this, _1));
};

void SceneManager::tick(const ros::TimerEvent& event) {
    /* Crit section data_mutex_ begin */
    std::unique_lock<std::mutex> lock{data_mutex_};
    if (last_cloud_ == nullptr || last_image_colour_ == nullptr) return;

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
    Percept curr_percept{image_cv, pcl_cloud};

    lock.unlock();
    /* Crit section data_mutex_ end */

    // Create a new scene
    {
        std::lock_guard<std::mutex> scene_lock{scene_.lock_};
        scene_.reset();
        scene_.setPercept(curr_percept);
    }
    yolo_generator_.run(scene_); 

    //plane_annotator_.run(scene_, scene_.getSofA());
    //size_annotator_.run(scene_, scene_.getSofA());
    colour_annotator_.run(scene_, scene_.getSofA());
    //shape_annotator_.run(scene_, scene_.getSofA());

    scene_.accept(scene_writer_);

}

} // namespace svs
