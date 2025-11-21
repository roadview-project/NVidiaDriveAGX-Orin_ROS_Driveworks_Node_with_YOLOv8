#include <ros/ros.h>
#include "drive_yolo/lidar_camera_fusion.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_camera_fusion_node");
    
    try {
        LidarCameraFusion fusion_node;
        ROS_INFO("LiDAR-Camera fusion node started");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Fusion node failed: %s", e.what());
        return -1;
    }
    
    return 0;
}