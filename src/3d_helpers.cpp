#include "spatial_visual_system/3d_helpers.h"

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <vector>
  
namespace svs {
/**
   * Extracts the pointcloud given a bounding box in the 2d image 
   * Expects the image to have the same aspect ratio as the point cloud
   */
  void extractCloudFromBbox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, int   imageCols, cv::Rect &boundary, std::vector<int>& cloud_indicies_out){
  
      // make a copy of the boundary
      cv::Rect bbox = boundary;
  
      // rescale the image coordinates to fit the point cloud resolution
      float scale = (float) imageCols / (float) cloud_in->width;
      bbox.x = (int) ((float) bbox.x / scale);
      bbox.y = (int) ((float) bbox.y / scale);
      bbox.height = (int) ((float) bbox.height / scale);
      bbox.width  = (int) ((float) bbox.width  / scale);
  
      // ROS_INFO("Extraction: bbox (%d, %d) (%d, %d), scale: %f", bbox.x, bbo  x.y, bbox.width, bbox.height, scale);
  
      // check boundary conditions one last time.
      if (bbox.x < 0) bbox.x = 0;
      if (bbox.y < 0) bbox.y = 0;
      if (bbox.x + bbox.width  >= cloud_in->width)  bbox.width  = cloud_in->width  - bbox.x - 1;
      if (bbox.y + bbox.height >= cloud_in->height) bbox.height = cloud_in->height - bbox.y - 1;
  
      // find all pcl points within the bounding region
      for(int r = bbox.y; r < bbox.y + bbox.height; ++r) {
          const pcl::PointXYZ *itP = &cloud_in->points[r* cloud_in->width + bbox.x];
          const pcl::PointXYZ *itBegin = &cloud_in->points[0];
          for(int c = bbox.x; c < bbox.x+bbox.width; ++c, ++itP) {
              if( std::isnan(itP->x) || std::isnan(itP->y) || std::isnan(itP->  z)) continue;
              cloud_indicies_out.push_back(std::distance(itBegin, itP));
          }
      }
  
      // ROS_INFO("Extraction: bbox: corner (%d, %d) size (%d, %d), num points  : %d", bbox.x, bbox.y, bbox.width, bbox.height, (int) cloud_out->size());
  
  }

} // namespace svs
