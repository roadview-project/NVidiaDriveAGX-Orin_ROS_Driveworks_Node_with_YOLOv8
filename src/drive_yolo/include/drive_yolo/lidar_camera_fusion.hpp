#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// TF2 includes for proper transform handling
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "drive_yolo/Detections.h"
#include "drive_yolo/Detection.h"
#include "drive_yolo/DetectionWithDistance.h"
#include "drive_yolo/DetectionsWithDistance.h"

#include <vector>
#include <memory>
#include <chrono>
#include <iomanip>
#include <sstream>

// Simple point structure
struct SimplePoint {
    float x, y, z;
    SimplePoint(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
};

class LidarCameraFusion {
public:
    LidarCameraFusion();
    ~LidarCameraFusion() = default;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Synchronized subscribers
    message_filters::Subscriber<drive_yolo::Detections> detection_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<drive_yolo::Detections, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    std::shared_ptr<Synchronizer> sync_;

    // Publishers
    ros::Publisher fused_detections_pub_;
    ros::Publisher debug_cloud_pub_;
    ros::Publisher detection_boxes_3d_pub_;
    ros::Publisher camera_info_pub_;
    ros::Timer camera_info_timer_;

    // TF2 for transform handling
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Frame names (configurable)
    std::string lidar_frame_;
    std::string camera_frame_;
    std::string base_frame_;
    
    // Camera calibration parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // Current transform (updated dynamically from TF)
    cv::Mat rotation_matrix_;
    cv::Mat translation_vector_;
    bool transform_valid_;
    ros::Time last_transform_update_;
    
    // Parameters
    bool debug_visualization_;
    bool use_tf_transforms_;
    double max_distance_;
    double min_distance_;
    double distance_filter_tolerance_;
    double transform_timeout_;

    // NEW: Synchronization parameters
    double sync_time_slop_;     // Time tolerance for message synchronization (seconds)
    int sync_queue_size_;       // Queue size for synchronizer buffer
    double time_offset_ms_;     // Manual time offset between camera and LiDAR (ms)

    double ground_height_threshold_;
    double lidar_height_above_ground_;
    bool enable_ground_filtering_;
    
    // NEW: Point clustering parameters
    double max_point_distance_from_center_;
    int min_points_for_object_;
    double point_density_threshold_;
    bool enable_point_clustering_;
    
    // NEW: Object size constraints
    double max_object_height_;
    double max_object_width_;
    double outlier_distance_threshold_;

    // ROI filtering parameters (ego vehicle coordinate system)
    double roi_x_min_;  // Minimum forward distance (m)
    double roi_x_max_;  // Maximum forward distance (m)
    double roi_y_min_;  // Minimum lateral distance (m, negative = right)
    double roi_y_max_;  // Maximum lateral distance (m, positive = left)
    bool enable_roi_filtering_;

    // NEW: Enhanced filtering methods
    std::vector<SimplePoint> filterPointsByCluster(
        const std::vector<SimplePoint>& candidate_points,
        const drive_yolo::Detection& detection);
    
    cv::Point3f calculateObjectDimensions(const std::vector<SimplePoint>& points);
    
    // Main callback
    void fusionCallback(const drive_yolo::Detections::ConstPtr& detections_msg,
                       const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    // TF-based transformation functions
    bool updateTransformFromTF(const ros::Time& timestamp);
    bool getTransformMatrix(const std::string& target_frame, 
                           const std::string& source_frame,
                           const ros::Time& timestamp,
                           cv::Mat& rotation, cv::Mat& translation);
    
    // Core fusion functions
    std::vector<cv::Point2f> projectPointsToImage(const std::vector<SimplePoint>& points);
    std::vector<SimplePoint> parsePointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    std::vector<SimplePoint> filterPointsByDistance(const std::vector<SimplePoint>& points);
    
    // Distance estimation functions
    double estimateObjectDistance(const drive_yolo::Detection& detection, 
                                const std::vector<SimplePoint>& points,
                                const std::vector<cv::Point2f>& projected_points);
    std::vector<SimplePoint> getPointsInBoundingBox(const drive_yolo::Detection& detection,
                                                   const std::vector<SimplePoint>& points,
                                                   const std::vector<cv::Point2f>& projected_points);
    cv::Point3f calculateObjectCenterEnhanced(const std::vector<SimplePoint>& points, const drive_yolo::Detection& detection);
    cv::Point3f calculateWeightedCenter(const std::vector<SimplePoint>& points);
    float calculatePointSpread(const std::vector<SimplePoint>& points) ;


    cv::Point3f calculateCenterWithClustering(const std::vector<SimplePoint>& points);

    float getEstimatedObjectDepth(int class_id) ;
    
    
    // 3D Visualization functions
    visualization_msgs::Marker create3DBoundingBoxMarker(
        const drive_yolo::Detection& detection,
        const cv::Point3f& center_3d,
        const std::vector<SimplePoint>& bbox_points,
        int marker_id,
        const std_msgs::Header& header);
    
    visualization_msgs::Marker createTextMarker(
        const drive_yolo::Detection& detection,
        const cv::Point3f& center_3d,
        double distance,
        int marker_id,
        const std_msgs::Header& header);
    
    void setMarkerColor(visualization_msgs::Marker& marker, int class_id, float confidence);
    
    // Camera Info publishing functions
    void publishCameraInfo();
    void cameraInfoTimerCallback(const ros::TimerEvent&);
    
    // Transform debugging
    void logCurrentTransform();
    void logTFTree();
    
    // Utility functions
    bool loadCalibrationParameters();
    cv::Point3f calculateObjectCenter(const std::vector<SimplePoint>& points);
    double calculateAverageDistance(const std::vector<SimplePoint>& points);
    double calculateMedianDistance(const std::vector<SimplePoint>& points);
    
    // Point cloud parsing helpers
    bool isValidPointCloudFormat(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    size_t getPointCloudSize(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
};