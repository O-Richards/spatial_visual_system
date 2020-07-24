
#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/pcl_base.h>

#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

class LineModDetector {
private:
    pcl::LINEMOD detector_;
    using Cloud = pcl::PointCloud<pcl::PointXYZRGBA>;

public:
    void add_object(std::string path) {
        // Load file
        Cloud::Ptr cloud{new Cloud{}};
        if (pcl::io::loadPCDFile(path, *cloud) < 0) {
            throw "Unable to read file: " + path;
        }

        std::cout << "Loaded " << path <<
            " points: " << cloud->width * cloud->height;

        
        // Calculate modalities
        pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
        color_grad_mod.setInputCloud (cloud);
        color_grad_mod.processInputData ();

        pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
        surface_norm_mod.setInputCloud (cloud);
        surface_norm_mod.processInputData ();

        std::vector<pcl::QuantizableModality*> modalities (2);
        modalities[0] = &color_grad_mod;
        modalities[1] = &surface_norm_mod;

        pcl::MaskMap mask{cloud->width, cloud->height};
        std::vector<pcl::MaskMap*> masks{&mask, &mask};

        pcl::PointXYZRGBA min_point;
        pcl::PointXYZRGBA max_point;
        pcl::getMinMax3D(*cloud, min_point, max_point);
        pcl::RegionXY region{};
        region.x = min_point.x;
        region.y = min_point.y;
        region.height = max_point.y - min_point.y;
        region.width = max_point.x - min_point.x;
        detector_.createAndAddTemplate(modalities, masks, region);
    }

    void save_templates(std::string path) {
        detector_.saveTemplates(path.c_str());
    }

};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " template_path\n";
        return -1;
    }

    LineModDetector detector;

    std::string input_path = argv[1];
    detector.add_object(input_path);
    detector.save_templates("templates.tmp");
}
