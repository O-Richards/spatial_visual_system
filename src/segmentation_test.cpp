#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/transforms.h>


int main(int argc, char** argv) {
    //pcl::visualization::CloudViewer viewer("viewer1");

    auto cloud_in = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::PCDReader reader;

    reader.read("clouds/table.pcd", *cloud_in);

    // downsample
    auto cloud_downsampled = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_downsampled);

}
