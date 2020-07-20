/*
 * Annotator to extract the size of an object and ground this attribute
 * Uses the bounding box from Yolo and a minimal 3D bounding box to estimate size
 */

#include "spatial_visual_system/sofa_annotator.h"
#include "spatial_visual_system/sofa.h"
#include "spatial_visual_system/3d_helpers.h"

#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


namespace svs {

#define DEBUG 1

void Size3DAnnotator::run(const Scene& scene, std::vector<SofA>& sofa) {
    auto scene_cloud = scene.getPercept().cloud_;
    #pragma omp parallel
    #pragma omp for
    for (auto& s : sofa) {
        std::lock_guard<decltype(s.lock_)> lock(s.lock_);
        auto sofa_cloud = boost::make_shared<svs::PointCloud>();
        for (const auto i : s.cloud_index_mask_) {
            sofa_cloud->push_back(scene_cloud->at(i));
        }

        // For a quick and dirty result:
        // Find the <x, y, z> coords for the top left and bottom right points
        
        // First remove outliers to not screw with size
        auto cloud_filtered = boost::make_shared<svs::PointCloud>();
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(sofa_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered);

        // Find bounding box of cloud area
        pcl::PointXYZ min;
        pcl::PointXYZ max;

        pcl::getMinMax3D(*cloud_filtered, min, max);

        // We are in the camera link (assert this)
        // So the 2D area is (maxx - minx)*(maxy-miny)
        if (cloud_filtered->header.frame_id != camera_link_frame_) {
            ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " Point cloud in frame " << cloud_filtered->header.frame_id <<
                    " not in expected frame " << camera_link_frame_);
        }
        auto area = (max.x - min.x)*(max.y-min.y);

        std::string semantic_label;
        double confidence;
        groundArea(area, semantic_label, confidence);

        // write out the info.
        s.annotations_["area"] = area;
        s.annotations_["size"] = semantic_label;
        s.annotations_["size_confidence"] = confidence;

#if DEBUG
        ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " sofa " << s.getId() << 
                " area " << area << " " << semantic_label << " conf: " << confidence);
        // Write out the filtered image
        pcl::PCDWriter writer{};
        std::ostringstream out_name;
        out_name << "/home/oli/cylinder_fits/sofa_" << s.getId();
        try {
            writer.write<svs::Point>(out_name.str() + "_stat_outlier_removed_filtered.pcd", *cloud_filtered);
        } catch (const std::exception& e) { 
            ROS_WARN_STREAM(__PRETTY_FUNCTION__ << e.what());
        }
#endif
    }
}

void Size3DAnnotator::groundArea(double area, std::string& semantic_label, double& confidence) const {
    // Situation for grounding:
    // area number line: -----|-----|------|----
    //                   small|med  |large |...
    //                   -----|--*--|--*---|
    // Find the % error quantified as (distance from center of nearest label to point) / measurement
    if (area < small_to_med_thresh_) {
        semantic_label = "small";
        confidence = std::abs(area - small_to_med_thresh_ / 2.0) / area;
    } else if (area < med_to_large_thresh) {
        semantic_label = "medium";
        auto mpt = (med_to_large_thresh - small_to_med_thresh_) / 2.0;
        confidence = std::abs(area - mpt) / area;
    } else if (area < large_to_v_large_thresh) {
        semantic_label = "large";
        auto mpt = (large_to_v_large_thresh - med_to_large_thresh) / 2.0;
        confidence = std::abs(area - mpt) / area;
    } else {
        semantic_label = "very large";
        confidence = std::abs(area - large_to_v_large_thresh / 2.0) / area;
    }
}

void Size3DAnnotator::read_params() {
}


} // namespace svs
