#include "spatial_visual_system/scene.h"

#include "spatial_visual_system/scene_writer.h"

#include "spatial_visual_system/3d_helpers.h"

#include <boost/filesystem.hpp>

namespace svs {

void Scene::accept(SceneWriter &writer) {
    writer.new_scene();
    for (auto& sofa : sofa_) {
        writer.write(sofa);
    }
}

void Scene::saveSofA(SofA& sofa, const std::string& dir) {
    // lock the SofA
    std::lock_guard<decltype(sofa.lock_)> sofa_lock{sofa.lock_};

    std::string img_path = dir + "/rgb.jpeg";
    cv::imwrite(img_path, sofa.percept_.rgb_->image);

    // extract out cloud
    auto sofa_cloud = boost::make_shared<PointCloud>();
    {
        //auto lock_scene = std::lock_guard<std::mutex>{lock_};
        for (const auto i : sofa.cloud_index_mask_) {
            sofa_cloud->push_back(percept_.cloud_->at(i));
        }
    }

    // save cloud
    std::string pcd_path = dir + "/cloud.pcd";
    pcl::io::savePCDFile(pcd_path, *sofa_cloud);

    // save description
    std::string annotations_path = dir + "/annotations.json";
    std::ofstream annotations_file(annotations_path);
    annotations_file << sofa.annotations_;
}

void Scene::save(const std::string& dir) {
    std::lock_guard<std::mutex> lock{lock_};
    cv::imwrite(dir + "/scene_rgb.jpeg", percept_.rgb_->image);
    for (auto& sofa : sofa_) {
        std::stringstream sofa_dir;
        sofa_dir << dir << "/sofa_" << sofa.getId();
        boost::filesystem::create_directory(sofa_dir.str());
        saveSofA(sofa, sofa_dir.str());
    }
}

} // namespace svs
