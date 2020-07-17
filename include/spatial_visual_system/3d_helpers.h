#ifndef SPATIAL_VISUAL_SYSTEM_3D_HELPERS_H_
#define SPATIAL_VISUAL_SYSTEM_3D_HELPERS_H_

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace svs {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using Normal = pcl::Normal;
using NormalCloud = pcl::PointCloud<Normal>;

void extractCloudFromBbox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, 
        int imageCols, cv::Rect& boundary, 
        std::vector<int>& cloud_indicies_out);

/*
void extractCloudFromIndicies(NormalCloud::ConstPtr cloud, 
        const std::vector<int>& indicies, NormalCloud& cloud_out) {
    for (const auto i : indicies) {
        cloud_out.push_back(cloud->at(i));
    }
}
*/

/*
void extractCloudFromIndicies(PointCloud::ConstPtr cloud, 
        const std::vector<int>& indicies, PointCloud& cloud_out) {
    for (const auto i : indicies) {
        cloud_out.push_back(cloud->at(i));
    }
}
*/

} // namespace svs
#endif
