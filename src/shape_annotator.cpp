#include "spatial_visual_system/sofa_annotator.h"
#include "spatial_visual_system/3d_helpers.h"

#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sstream>

namespace svs {

#define DEBUG 1

static const double CYLINDER_INLIERS_PROPORTION_THRESH = 0.5;

ShapeAnnotator::ShapeAnnotator(ros::NodeHandle& nh) {
    
}

void ShapeAnnotator::run(const Scene& scene, std::vector<SofA>& sofa_list) {
    // Setup the filters
    for (auto& sofa : sofa_list) {
        std::lock_guard<decltype(sofa.lock_)> lock{sofa.lock_};

        // Extract the needed data of the SOFA
        PointCloud::Ptr sofa_cloud = boost::make_shared<PointCloud>();
        auto scene_cloud = scene.getPercept().cloud_;
        for (const auto i : sofa.cloud_index_mask_) {
            sofa_cloud->push_back(scene_cloud->at(i));
        }
        
        NormalCloud::Ptr sofa_normals = boost::make_shared<NormalCloud>();
        auto scene_normals = scene.getPercept().cloud_normals_;
        if (scene_normals->size() != scene_cloud->size()) {
            std::ostringstream msg{};
            msg << "Scene cloud size (" << scene_cloud->height << ", " << scene_cloud->width <<
                ") Not equal to normal cloud size (" << scene_normals->height << ", " <<
                scene_normals->width << ")";
            throw std::runtime_error(msg.str());
        }

        for (const auto i : sofa.cloud_index_mask_) {
            sofa_normals->push_back(scene_normals->at(i));
        }


        // Apply RanSAC to try fit a cylinder
        auto coeffs_cylinder = boost::make_shared<pcl::ModelCoefficients>();
        auto cylinder_inliers = boost::make_shared<pcl::PointIndices>();
        // Extract adapted from RoboSherlock
        pcl::SACSegmentationFromNormals<svs::Point, svs::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.03);
        seg.setRadiusLimits(0, 0.1);
        seg.setInputCloud(sofa_cloud);
        seg.setInputNormals(sofa_normals);

        // Obtain the cylinder inliers and coefficients
        seg.segment(*cylinder_inliers, *coeffs_cylinder);

        auto num_inliers = cylinder_inliers->indices.size();
        auto proportion_inliers = num_inliers / static_cast<double>(sofa_cloud->size());
        
        // Cylinder model params
        // See https://www.programmersought.com/article/4958567145/ for what these mean
        /*
        float x0 = coeffs_cylinder->values[0];
        float y0 = coeffs_cylinder->values[1];
        float z0 = coeffs_cylinder->values[2];
        float l = coeffs_cylinder->values[3];
        float m= coeffs_cylinder->values[4];
        float n = coeffs_cylinder->values[5];
        float r0 = coeffs_cylinder->values[6];
        */
        if (proportion_inliers > CYLINDER_INLIERS_PROPORTION_THRESH) {
            // Decide it is a cylinder
            // TODO: Add shape to DB!
            sofa.annotations_["shape"] = "cylinder";
            sofa.annotations_["description"] += "cylinder,";
        }

#if DEBUG
        ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " extract cylinder from sofa " << sofa.getId() << " found " << num_inliers << " inliers "
                << proportion_inliers*100 << "\% of cloud");
        if (num_inliers > 0) {

            auto cylinder_cloud = boost::make_shared<svs::PointCloud>();
            pcl::ExtractIndices<svs::Point> extract{};
            extract.setInputCloud(sofa_cloud);
            extract.setIndices(cylinder_inliers);
            extract.setNegative(false);
            extract.filter(*cylinder_cloud);

            std::ostringstream out_name;
            out_name << "/home/oli/cylinder_fits/sofa_" << sofa.getId();

            pcl::PCDWriter writer{};
            try {
                writer.write<svs::Point>(out_name.str() + "_cylinder_filtered.pcd", *cylinder_cloud);
                writer.write<svs::Point>(out_name.str() + ".pcd", *sofa_cloud);
            } catch (const std::exception& e) { 
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << e.what());
            }
        }

#endif
    }
}


} // namespace svs
