#include "spatial_visual_system/sofa_annotator.h"

#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    
    auto scene_cloud = scene.getPercept().cloud_;
    // Check frame is what we expect
    if (scene_cloud->header.frame_id != working_frame_) {
#if DEBUG
        ROS_INFO_STREAM(__FUNCTION__ << " converting scene cloud to frame " << working_frame_);
#endif
        std::string transform_error{};
        auto header = pcl_conversions::fromPCL(scene_cloud->header);
        auto tf_avail = tf_listener_.waitForTransform(working_frame_, header.frame_id, 
                header.stamp, ros::Duration{1},
                ros::Duration{0.1}, &transform_error);

        if (!tf_avail) ROS_ERROR_STREAM(__FUNCTION__ << " wait for transform failed " << transform_error);

        auto transformed_cloud = boost::make_shared<PointCloud>();
        pcl_ros::transformPointCloud(working_frame_, *scene_cloud, *transformed_cloud, tf_listener_);
        scene_cloud = transformed_cloud;
    }

#if DEBUG
    ROS_INFO_STREAM(__FUNCTION__ << " fitting plane to cloud in frame " << scene_cloud->header.frame_id);
#endif

    auto ground_removed = boost::make_shared<PointCloud>();
    //*ground_removed = *scene_cloud;
    removeGround(scene_cloud, ground_removed);

    auto filtered_cloud = boost::make_shared<PointCloud>();
    filtered_cloud = ground_removed;
    /*
    pcl::StatisticalOutlierRemoval<Point> stat_outlier_remover;
    // number (k) neighbors that we will compare the selected point against
    stat_outlier_remover.setMeanK(50);
    // number (k) neighbors that we will compare the selected point against
    stat_outlier_remover.setStddevMulThresh(1.0);
    stat_outlier_remover.setInputCloud(ground_removed);
    stat_outlier_remover.filter(*filtered_cloud);
    */

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
    seg.setOptimizeCoefficients(true);
    
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

    seg.setMaxIterations(1000); 
    seg.setMethodType(pcl::SAC_RANSAC);
    
    // distance in m
    // peter: 0.01 (10mm) might be slightly too aggressive...
    // actually, 10mm may not be aggressive enough.. trying 15mm
    seg.setDistanceThreshold (0.005);

    seg.setInputCloud(filtered_cloud);
    seg.segment (*inliers, *coefficients);

#if DEBUG
    ROS_INFO_STREAM(__FUNCTION__ << " found " << inliers->indices.size() << " plane inliers");

    // Create cloud to publish inliers
    auto cloud_pub = boost::make_shared<PointCloud>();
    pcl::copyPointCloud(*filtered_cloud, inliers->indices, *cloud_pub);
    plane_pub_.publish(cloud_pub);
    
#endif

    // Remove plane from SofA
    // Note that we carefully kept the input point cloud in the same dimensions etc.
    // So we can just directly use the indicies
    for (auto& sofa : sofa_list) {
        std::lock_guard<decltype(sofa.lock_)> lock{sofa.lock_};
        std::vector<int> diff{};
        std::set_difference(sofa.cloud_index_mask_.begin(), sofa.cloud_index_mask_.end(),
                inliers->indices.begin(), inliers->indices.end(),
                std::inserter(diff, diff.begin()));
        sofa.cloud_index_mask_ = diff;
    }

    // TODO
    // Add this plane as a SofA
}

void PlaneAnnotator::removeGround(PointCloud::ConstPtr in_cloud, 
        PointCloud::Ptr out_cloud) {

    *out_cloud = *in_cloud;
    // Replace positions of floor with NaN's
    int num_removed = 0;
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    for (auto& p : *out_cloud) {
        if (p.z < GROUND_FILTER_MIN_HEIGHT) {
            p.x = bad_point;
            p.y = bad_point;
            p.z = bad_point;

            num_removed++;
        }
    }

#if DEBUG
    /*
    auto point_is_nan = [](const auto& p) {
        const float bad_point = std::numeric_limits<float>::quiet_NaN();
        return p.x == bad_point && p.y == bad_point && p.z == bad_point;
    };

    int num_nan_orig = std::count_if(in_cloud->begin(), in_cloud->end(), point_is_nan);
    int num_nan = std::count_if(out_cloud->begin(), out_cloud->end(), point_is_nan);
    */
    ROS_INFO_STREAM("Removed " << num_removed << " ground points");
#endif

    /*
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(GROUND_FILTER_MIN_HEIGHT, FLT_MAX); // remove 5cm from ground
    pass.filter(*out_cloud);
    */
}


} // namespace svs
