#ifndef SPATIAL_VISUAL_SYSTEM_3D_HELPERS_H_
#define SPATIAL_VISUAL_SYSTEM_3D_HELPERS_H_

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace svs {
void extractCloudFromBbox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, 
        int imageCols, cv::Rect& boundary, 
        std::vector<int>& cloud_indicies_out);

}
#endif
