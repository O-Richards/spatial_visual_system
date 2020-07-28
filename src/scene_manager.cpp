#include "spatial_visual_system/scene_manager.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <boost/filesystem.hpp>

namespace svs {

#define DEBUG true

unsigned int SceneManager::scene_no_ = 0;

SceneManager::SceneManager(ros::NodeHandle& nh):
    nh_{nh}, yolo_generator_{nh_}, plane_annotator_{nh_}, shape_annotator_{nh_}, size_annotator_{nh_}, sift_annotator_{nh_}, scene_writer_{nh_}
{
    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
    image_transport::TransportHints hints("raw");

    sub_image_colour_ = 
        std::make_unique<image_transport::SubscriberFilter>(it, "image", queue_size_, hints);

    sub_cloud_ = 
        std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, "points", queue_size_);

    sub_detections_ = 
        std::make_unique<message_filters::Subscriber<unsw_vision_msgs::DetectionList>>(nh, "processed_yolo_detections", queue_size_);

    sub_sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_), *sub_image_colour_, *sub_cloud_, *sub_detections_);
    sub_sync_->registerCallback(boost::bind(&SceneManager::sync_cb, this, _1, _2, _3));

    // setup timer
    //tick_timer_ = nh_.createTimer(ros::Duration(1/svs_freq_), boost::bind(&SceneManager::tick, this, _1));
};

void SceneManager::tick(const ros::TimerEvent& event) {
    tick();
}

void SceneManager::tick() {
    /* Crit section data_mutex_ begin */
    std::unique_lock<std::mutex> lock{data_mutex_};
    auto start = std::chrono::high_resolution_clock::now();

    if (last_cloud_ == nullptr || last_image_colour_ == nullptr || last_detections_ == nullptr) return;
    if (last_cloud_->height * last_cloud_->width <= 0 || last_image_colour_->data.size() <= 0) return;

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
    unsw_vision_msgs::DetectionList curr_detections = *last_detections_;

    // reset to nullptr so next tick will skip if needed
    last_cloud_ = nullptr;
    last_image_colour_ = nullptr;
    last_detections_ = nullptr;

    lock.unlock();
    /* Crit section data_mutex_ end */

    // Create a new scene
    {
        std::lock_guard<std::mutex> scene_lock{scene_.lock_};
        scene_.reset();
        scene_.setPercept(curr_percept);
        scene_.setDetections(curr_detections);
    }
    yolo_generator_.run(scene_); 
    
    // These can be done in parrallel
    auto now = std::chrono::high_resolution_clock::now;
    auto sift_begin = now();
    sift_annotator_.run(scene_, scene_.getSofA());
    auto sift_end = now();
    //plane_annotator_.run(scene_, scene_.getSofA());
    auto size_begin = now();
    size_annotator_.run(scene_, scene_.getSofA());
    auto size_end = now();
    auto colour_begin = now();
    colour_annotator_.run(scene_, scene_.getSofA());
    auto colour_end = now();
    auto shape_begin = now();
    scene_.getPercept().calculateCloudNormals(); // This is absolutely killing the CPU
    shape_annotator_.run(scene_, scene_.getSofA());
    auto shape_end = now();

    auto write_begin = now();
    scene_.accept(scene_writer_);
    auto write_end = now();

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

#if DEBUG
    ROS_INFO_STREAM("SceneManager::tick() took " << elapsed_t.count() << "ms\n"
            << "\t sift time: " << std::chrono::duration_cast<std::chrono::milliseconds>(sift_end - sift_begin).count() << "ms\n"
            << "\t size time: " << std::chrono::duration_cast<std::chrono::milliseconds>(size_end - size_begin).count() << "ms\n"
            << "\t colour time: " << std::chrono::duration_cast<std::chrono::milliseconds>(colour_end - colour_begin).count() << "ms\n"
            << "\t shape time: " << std::chrono::duration_cast<std::chrono::milliseconds>(shape_end - shape_begin).count() << "ms\n"
            << "\t writing scene: " << std::chrono::duration_cast<std::chrono::milliseconds>(write_end - write_begin).count() << "ms\n");

    // save out scene
    std::stringstream scene_dir;
    scene_dir << scene_save_dir_ << "/scene_" << scene_no_;
    try {
        boost::filesystem::create_directory(scene_dir.str());
        scene_.save(scene_dir.str());
    } catch (std::exception e) {
        ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " failed to save scene " << e.what());
    }
#endif

    scene_no_++;
}



} // namespace svs
