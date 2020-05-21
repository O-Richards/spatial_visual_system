#include <ros/ros.h>
#include <ros/node_handle.h>

#include "spatial_visual_system/scene.h"
#include "spatial_visual_system/yolo.h"

class SceneManager {
    private:
    ros::NodeHandle& nh_;
    svs::Scene scene_;
    svs::YoloGenerator yolo_generator_;
    double svs_freq_ = 10;
    ros::Rate rate_controller_;
    public:
    SceneManager(ros::NodeHandle& nh): nh_{nh}, yolo_generator_{nh_}, rate_controller_{svs_freq_} {};
    void tick() {yolo_generator_.run(scene_); rate_controller_.sleep();};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "spatial_visual_system");
    ros::NodeHandle nh{"~"};
    ROS_INFO("Beep boop... starting up spatial_visual_system");

    SceneManager scene_manager{nh};

    ROS_INFO("Ticking scene_manager. No explicit spins!");
    while(ros::ok()) {
        scene_manager.tick();
        ros::spinOnce();
    }

    ROS_INFO("spatial_visual_system shutting down!");
}
