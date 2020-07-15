#include "spatial_visual_system/sofa_annotator.h"

#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_ros/transforms.h>

namespace svs {

#define DEBUG 1
static const float GROUND_FILTER_MIN_HEIGHT = 0.05; // 5cm

PlaneAnnotator::PlaneAnnotator(ros::NodeHandle& nh) :
    nh_{&nh}
{
    plane_pub_ = nh_->advertise<PointCloud>("plane_annotator_debug", 1);
}

void PlaneAnnotator::run(const Scene& scene, std::vector<SofA>& sofa_list) {
    //std::lock_guard<decltype(scene.lock_)> lock{scene.lock_};
    // We first remove the ground plane
    // Then we find the largest plane in the scene and remove this
    // Extract out the dominant plane
    
    auto ground_removed = boost::make_shared<PointCloud>();
    auto scene_cloud = scene.getPercept().cloud_;
    // Check frame is what we expect
    if (scene_cloud->header.frame_id != working_frame_) {
        auto transformed_cloud = boost::make_shared<PointCloud>();
        pcl_ros::transformPointCloud(working_frame_, *scene_cloud, *transformed_cloud, tf_listener_);
        scene_cloud = transformed_cloud;
    }
    removeGround(scene_cloud, ground_removed);

    // Find plane in cloud
    /*auto ransac_model = 
        boost::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(ground_removed);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac{ransac_model};

    std::vector<int> inliers;
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
    */
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
    pcl::SACSegmentation<Point> seg;
    
    // Optional
    seg.setOptimizeCoefficients (true);
    
    // Mandatory
    // should ID planes perpendicular to the z axis (ex: tabletops)
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    
    // will ID any plane
    //seg.setModelType (pcl::SACMODEL_PLANE);
    
    // set axis and eps
    // hint at z axis
    seg.setAxis(Eigen::Vector3f(0,0,1));
    
    // 5 degree in rads
    seg.setEpsAngle((5.0*3.14)/180.0); 

    seg.setMaxIterations(500); 
    seg.setMethodType (pcl::SAC_RANSAC);
    
    // distance in m
    // peter: 0.01 (10mm) might be slightly too aggressive...
    // actually, 10mm may not be aggressive enough.. trying 15mm
    seg.setDistanceThreshold (0.015);

    seg.setInputCloud(ground_removed);
    seg.segment (*inliers, *coefficients);

#if DEBUG
    ROS_INFO_STREAM(__FUNCTION__ << " found " << inliers->indices.size() << " plane inliers");

    // Create cloud to publish inliers
    auto cloud_pub = boost::make_shared<PointCloud>();
    pcl::copyPointCloud(*scene_cloud, inliers->indices, *cloud_pub);
    plane_pub_.publish(cloud_pub);
#endif

    // Remove plane from SofA
    for (auto& sofa : sofa_list) {
        std::lock_guard<decltype(sofa.lock_)> lock{sofa.lock_};
        std::vector<int> diff{};
        std::set_difference(sofa.cloud_index_mask_.begin(), sofa.cloud_index_mask_.end(),
                inliers->indices.begin(), inliers->indices.end(),
                std::inserter(diff, diff.begin()));
        sofa.cloud_index_mask_ = diff;
    }

    // Add this plane as a SofA
    

}

void PlaneAnnotator::removeGround(PointCloud::ConstPtr in_cloud, 
        PointCloud::Ptr out_cloud) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(GROUND_FILTER_MIN_HEIGHT, FLT_MAX); // remove 5cm from ground
    pass.filter(*out_cloud);
}


} // namespace svs
