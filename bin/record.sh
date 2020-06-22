#!/bin/bash

rosbag record /yolo2_node/object_detections \
    /hsrb/head_rgbd_sensor/rgb/image_raw /hsrb/head_rgbd_sensor/rgb/camera_info \
    /hsrb/head_rgbd_sensor/depth_registered/image_raw /hsrb/head_rgbd_sensor/depth_registered/camera_info
