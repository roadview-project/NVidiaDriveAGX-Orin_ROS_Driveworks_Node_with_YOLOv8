#include "drive_yolo/lidar_camera_fusion.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

LidarCameraFusion::LidarCameraFusion() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_) {
    
    // Get parameters
    pnh_.param<bool>("debug_visualization", debug_visualization_, true);
    pnh_.param<bool>("use_tf_transforms", use_tf_transforms_, true);
    pnh_.param<double>("max_distance", max_distance_, 50.0);
    pnh_.param<double>("min_distance", min_distance_, 0.5);
    pnh_.param<double>("distance_filter_tolerance", distance_filter_tolerance_, 2.0);
    pnh_.param<double>("transform_timeout", transform_timeout_, 0.1);  // 100ms timeout

    // NEW: Synchronization parameters for handling camera/LiDAR timing mismatch
    pnh_.param<double>("sync_time_slop", sync_time_slop_, 0.1);  // 100ms tolerance for matching
    pnh_.param<int>("sync_queue_size", sync_queue_size_, 20);    // Buffer size for synchronizer
    pnh_.param<double>("time_offset_ms", time_offset_ms_, 0.0);   // Manual time offset (camera relative to LiDAR)

    // Frame names (configurable)
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
    pnh_.param<std::string>("camera_frame", camera_frame_, "camera_entron");  // Your actual camera frame
    pnh_.param<std::string>("base_frame", base_frame_, "car_rear_axle");

    pnh_.param<double>("ground_height_threshold", ground_height_threshold_, 0.5);  // 0.5m above ground
    pnh_.param<double>("lidar_height_above_ground", lidar_height_above_ground_, 1.383);  // 1.041 + 0.342 = 1.383m
    pnh_.param<bool>("enable_ground_filtering", enable_ground_filtering_, true);
    
    // NEW: Point clustering parameters for better object detection
    pnh_.param<double>("max_point_distance_from_center", max_point_distance_from_center_, 3.0);  // 3m max from bbox center
    pnh_.param<int>("min_points_for_object", min_points_for_object_, 8);  // Need at least 8 points
    pnh_.param<double>("point_density_threshold", point_density_threshold_, 0.1);  // Points per square meter
    pnh_.param<bool>("enable_point_clustering", enable_point_clustering_, true);
    
    // NEW: Advanced filtering
    pnh_.param<double>("max_object_height", max_object_height_, 4.0);  // 4m max object height
    pnh_.param<double>("max_object_width", max_object_width_, 3.0);   // 3m max object width
    pnh_.param<double>("outlier_distance_threshold", outlier_distance_threshold_, 2.0);  // Remove outliers 2m+ away

    // ROI filtering parameters (default: 2m right, 7m left, 50m forward)
    pnh_.param<bool>("enable_roi_filtering", enable_roi_filtering_, true);
    pnh_.param<double>("roi_x_min", roi_x_min_, 0.0);      // Minimum forward distance
    pnh_.param<double>("roi_x_max", roi_x_max_, 50.0);     // Maximum forward distance
    pnh_.param<double>("roi_y_min", roi_y_min_, -2.0);     // Right side limit (negative)
    pnh_.param<double>("roi_y_max", roi_y_max_, 7.0);      // Left side limit (positive)

    ROS_INFO("=== ENHANCED LIDAR-CAMERA FUSION PARAMETERS ===");
    ROS_INFO("Synchronization:");
    ROS_INFO("  Time slop: %.0f ms", sync_time_slop_ * 1000.0);
    ROS_INFO("  Queue size: %d", sync_queue_size_);
    if (std::abs(time_offset_ms_) > 0.01) {
        ROS_INFO("  Time offset: %+.1f ms (camera relative to LiDAR)", time_offset_ms_);
    }
    ROS_INFO("Ground filtering: %s", enable_ground_filtering_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Ground height threshold: %.2f m above ground", ground_height_threshold_);
    ROS_INFO("LiDAR height above ground: %.2f m", lidar_height_above_ground_);
    ROS_INFO("Point clustering: %s", enable_point_clustering_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Max point distance from center: %.1f m", max_point_distance_from_center_);
    ROS_INFO("Min points for object: %d", min_points_for_object_);
    ROS_INFO("Max object dimensions: %.1f x %.1f x %.1f m", max_object_width_, max_object_width_, max_object_height_);
    ROS_INFO("ROI filtering: %s", enable_roi_filtering_ ? "ENABLED" : "DISABLED");
    if (enable_roi_filtering_) {
        ROS_INFO("  Forward: %.1f to %.1f m", roi_x_min_, roi_x_max_);
        ROS_INFO("  Lateral: %.1f to %.1f m (right to left)", roi_y_min_, roi_y_max_);
    }
    ROS_INFO("==============================================");
    
    transform_valid_ = false;

    // Load calibration parameters
    if (!loadCalibrationParameters()) {
        ROS_FATAL("Failed to load calibration parameters");
        ros::shutdown();
        return;
    }

    // Setup synchronized subscribers with configurable parameters
    detection_sub_.subscribe(nh_, "/detections", sync_queue_size_);
    pointcloud_sub_.subscribe(nh_, "/lidar/points", sync_queue_size_);

    // Create synchronizer with configurable time slop
    sync_.reset(new Synchronizer(SyncPolicy(sync_queue_size_), detection_sub_, pointcloud_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(sync_time_slop_));
    sync_->registerCallback(boost::bind(&LidarCameraFusion::fusionCallback, this, _1, _2));

    // Setup publishers
    fused_detections_pub_ = nh_.advertise<drive_yolo::DetectionsWithDistance>("/detections_with_distance", 10);
    detection_boxes_3d_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detection_boxes_3d", 10);
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera_info_fusion", 10);
    
    if (debug_visualization_) {
        debug_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug_filtered_cloud", 10);
    }

    // Publish camera info and set up timer
    publishCameraInfo();
    camera_info_timer_ = nh_.createTimer(ros::Duration(1.0), &LidarCameraFusion::cameraInfoTimerCallback, this);

    // Wait a bit for TF buffer to fill
    ros::Duration(2.0).sleep();
    
    // Try to get initial transform
    if (use_tf_transforms_) {
        ROS_INFO("Waiting for transform from %s to %s...", lidar_frame_.c_str(), camera_frame_.c_str());
        
        try {
            // Wait for transform to become available
            if (tf_buffer_.canTransform(camera_frame_, lidar_frame_, ros::Time(0), ros::Duration(5.0))) {
                updateTransformFromTF(ros::Time(0));
                ROS_INFO("TF transform loaded successfully!");
                logCurrentTransform();
            } else {
                ROS_ERROR("Failed to get transform from %s to %s", lidar_frame_.c_str(), camera_frame_.c_str());
                ROS_ERROR("Available frames:");
                logTFTree();
                ROS_ERROR("Falling back to manual calibration parameters");
                use_tf_transforms_ = false;
            }
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("TF Exception: %s", ex.what());
            ROS_ERROR("Falling back to manual calibration");
            use_tf_transforms_ = false;
        }
    }

    ROS_INFO("=== LIDAR-CAMERA FUSION INITIALIZED (TF-Based) ===");
    ROS_INFO("Using TF transforms: %s", use_tf_transforms_ ? "YES" : "NO");
    ROS_INFO("LiDAR frame: %s", lidar_frame_.c_str());
    ROS_INFO("Camera frame: %s", camera_frame_.c_str());
    ROS_INFO("Base frame: %s", base_frame_.c_str());
    ROS_INFO("Max distance: %.1f m", max_distance_);
    ROS_INFO("Min distance: %.1f m", min_distance_);
    ROS_INFO("Debug visualization: %s", debug_visualization_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Subscribing to: /detections, /lidar/points");
    ROS_INFO("Publishing to: /detections_with_distance, /detection_boxes_3d, /camera_info_fusion");
    ROS_INFO("=====================================================");
}

bool LidarCameraFusion::updateTransformFromTF(const ros::Time& timestamp) {
    try {
        // Get transform from lidar to camera
        geometry_msgs::TransformStamped transform_stamped;
        
        if (timestamp == ros::Time(0)) {
            // Use latest available transform
            transform_stamped = tf_buffer_.lookupTransform(camera_frame_, lidar_frame_, ros::Time(0));
        } else {
            // Use transform at specific time (with timeout)
            transform_stamped = tf_buffer_.lookupTransform(camera_frame_, lidar_frame_, timestamp, 
                                                          ros::Duration(transform_timeout_));
        }
        
        // Convert to OpenCV matrices
        return getTransformMatrix(camera_frame_, lidar_frame_, timestamp, rotation_matrix_, translation_vector_);
        
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "Failed to get transform: %s", ex.what());
        return false;
    }
}

bool LidarCameraFusion::getTransformMatrix(const std::string& target_frame, 
                                          const std::string& source_frame,
                                          const ros::Time& timestamp,
                                          cv::Mat& rotation, cv::Mat& translation) {
    try {
        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
            target_frame, source_frame, timestamp, ros::Duration(transform_timeout_));
        
        // Extract translation
        translation = (cv::Mat_<double>(3, 1) << 
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);
        
        // Extract rotation (quaternion to rotation matrix)
        tf2::Quaternion quat(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y, 
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        
        tf2::Matrix3x3 tf2_rotation(quat);
        
        // Convert tf2::Matrix3x3 to cv::Mat
        rotation = (cv::Mat_<double>(3, 3) << 
            tf2_rotation[0][0], tf2_rotation[0][1], tf2_rotation[0][2],
            tf2_rotation[1][0], tf2_rotation[1][1], tf2_rotation[1][2],
            tf2_rotation[2][0], tf2_rotation[2][1], tf2_rotation[2][2]);
        
        transform_valid_ = true;
        last_transform_update_ = ros::Time::now();
        
        return true;
        
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(5.0, "Transform lookup failed: %s", ex.what());
        transform_valid_ = false;
        return false;
    }
}

void LidarCameraFusion::logCurrentTransform() {
    if (!transform_valid_) {
        ROS_WARN("No valid transform available");
        return;
    }
    
    ROS_INFO("=== CURRENT LIDAR-TO-CAMERA TRANSFORM (from TF) ===");
    ROS_INFO("Translation:");
    ROS_INFO("  X: %.6f m", translation_vector_.at<double>(0, 0));
    ROS_INFO("  Y: %.6f m", translation_vector_.at<double>(1, 0));
    ROS_INFO("  Z: %.6f m", translation_vector_.at<double>(2, 0));
    
    ROS_INFO("Rotation Matrix:");
    for (int i = 0; i < 3; i++) {
        ROS_INFO("  [%8.6f %8.6f %8.6f]",
                 rotation_matrix_.at<double>(i, 0),
                 rotation_matrix_.at<double>(i, 1), 
                 rotation_matrix_.at<double>(i, 2));
    }
    
    // Convert rotation matrix back to RPY for human understanding
    tf2::Matrix3x3 tf2_rot(
        rotation_matrix_.at<double>(0, 0), rotation_matrix_.at<double>(0, 1), rotation_matrix_.at<double>(0, 2),
        rotation_matrix_.at<double>(1, 0), rotation_matrix_.at<double>(1, 1), rotation_matrix_.at<double>(1, 2),
        rotation_matrix_.at<double>(2, 0), rotation_matrix_.at<double>(2, 1), rotation_matrix_.at<double>(2, 2));
    
    double roll, pitch, yaw;
    tf2_rot.getRPY(roll, pitch, yaw);
    
    ROS_INFO("Rotation (RPY):");
    ROS_INFO("  Roll:  %.1f° (%.3f rad)", roll * 180.0/M_PI, roll);
    ROS_INFO("  Pitch: %.1f° (%.3f rad)", pitch * 180.0/M_PI, pitch);
    ROS_INFO("  Yaw:   %.1f° (%.3f rad)", yaw * 180.0/M_PI, yaw);
    ROS_INFO("Last updated: %.3f seconds ago", (ros::Time::now() - last_transform_update_).toSec());
    ROS_INFO("==================================================");
}

void LidarCameraFusion::logTFTree() {
    // Get all frames in the TF tree (compatible with different TF2 versions)
    std::vector<std::string> frame_strings;
    
    try {
        // Try the newer method first
        tf_buffer_._getFrameStrings(frame_strings);
    } catch (...) {
        // Fallback: manually check common frames
        std::vector<std::string> common_frames = {
            "car_rear_axle", "lidar", "camera_entron", "camera", 
            "base_link", "base_footprint", "odom", "map"
        };
        
        for (const auto& frame : common_frames) {
            try {
                if (tf_buffer_.canTransform("car_rear_axle", frame, ros::Time(0), ros::Duration(0.1))) {
                    frame_strings.push_back(frame);
                }
            } catch (...) {
                // Frame doesn't exist, skip
            }
        }
    }
    
    ROS_INFO("=== AVAILABLE TF FRAMES ===");
    if (frame_strings.empty()) {
        ROS_INFO("  (Could not list frames - TF2 version compatibility issue)");
        ROS_INFO("  Try: rosrun tf2_tools view_frames.py");
    } else {
        for (const auto& frame : frame_strings) {
            ROS_INFO("  - %s", frame.c_str());
        }
    }
    
    ROS_INFO("Looking for: %s -> %s", lidar_frame_.c_str(), camera_frame_.c_str());
    ROS_INFO("===========================");
}



bool LidarCameraFusion::loadCalibrationParameters() {
    // Load camera intrinsic parameters
    std::vector<double> camera_matrix_data, dist_coeffs_data;
    
    if (!pnh_.getParam("camera_matrix", camera_matrix_data) || camera_matrix_data.size() != 9) {
        ROS_ERROR("Failed to load camera_matrix parameter (should be 9 values)");
        return false;
    }
    
    if (!pnh_.getParam("distortion_coefficients", dist_coeffs_data) || dist_coeffs_data.size() < 4) {
        ROS_ERROR("Failed to load distortion_coefficients parameter (should be at least 4 values)");
        return false;
    }
    
    // Convert to OpenCV matrices
    camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
    dist_coeffs_ = cv::Mat(1, dist_coeffs_data.size(), CV_64F, dist_coeffs_data.data()).clone();
    
    ROS_INFO("Camera calibration loaded successfully");
    ROS_INFO("Camera matrix (fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f)", 
             camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(1,1),
             camera_matrix_.at<double>(0,2), camera_matrix_.at<double>(1,2));
    
    return true;
}

void LidarCameraFusion::fusionCallback(const drive_yolo::Detections::ConstPtr& detections_msg,
                                      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {

    auto fusion_start = std::chrono::high_resolution_clock::now();

    // Log timing information for synchronization debugging
    double time_diff_ms = (detections_msg->header.stamp - cloud_msg->header.stamp).toSec() * 1000.0;
    ROS_DEBUG("Sync: Camera-LiDAR offset = %+.1f ms (Camera: %.3f, LiDAR: %.3f)",
              time_diff_ms, detections_msg->header.stamp.toSec(), cloud_msg->header.stamp.toSec());

    // Apply manual time offset if configured
    if (std::abs(time_offset_ms_) > 0.01) {
        ROS_DEBUG_THROTTLE(5.0, "Applying time offset: %+.1f ms", time_offset_ms_);
    }

    // Update transform from TF if enabled
    if (use_tf_transforms_) {
        if (!updateTransformFromTF(cloud_msg->header.stamp)) {
            ROS_WARN_THROTTLE(5.0, "Failed to get current transform, using last valid transform");
            if (!transform_valid_) {
                ROS_ERROR_THROTTLE(10.0, "No valid transform available, skipping fusion");
                return;
            }
        }
    }
    
    // Check if point cloud format is valid
    if (!isValidPointCloudFormat(cloud_msg)) {
        ROS_WARN("Invalid point cloud format");
        return;
    }

    // Parse and filter point cloud
    std::vector<SimplePoint> all_points = parsePointCloud(cloud_msg);
    if (all_points.empty()) {
        ROS_WARN("No points in point cloud");
        return;
    }

    std::vector<SimplePoint> filtered_points = filterPointsByDistance(all_points);
    
    ROS_DEBUG("Point cloud: %zu total points, %zu after filtering", 
             all_points.size(), filtered_points.size());

    // Project LiDAR points to image coordinates
    std::vector<cv::Point2f> projected_points = projectPointsToImage(filtered_points);
    
    // Create output message
    drive_yolo::DetectionsWithDistance output_msg;
    output_msg.header = detections_msg->header;
    output_msg.inference_time_ms = detections_msg->inference_time_ms;
    output_msg.total_detections = detections_msg->total_detections;
    
    int detections_with_distance = 0;
    int processed_detections = 0;  // For logging
    
    // Prepare 3D visualization markers
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    
    // Process each detection
    for (const auto& detection : detections_msg->detections) {
        processed_detections++;
        
        // Estimate distance using LiDAR points
        double distance = estimateObjectDistance(detection, filtered_points, projected_points);
        
        // ONLY add detections that have valid distance information
        if (distance > 0) {  // Only process valid detections
            drive_yolo::DetectionWithDistance det_with_dist;
            det_with_dist.detection = detection;
            det_with_dist.distance = distance;
            det_with_dist.distance_confidence = 0.8;
            detections_with_distance++;
            
            // Get points in bounding box for additional info
            std::vector<SimplePoint> bbox_points = getPointsInBoundingBox(detection, filtered_points, projected_points);
            det_with_dist.lidar_points_count = bbox_points.size();
            
            if (!bbox_points.empty()) {
                cv::Point3f center = calculateObjectCenter(bbox_points);
                det_with_dist.object_center_3d.x = center.x;
                det_with_dist.object_center_3d.y = center.y;
                det_with_dist.object_center_3d.z = center.z;
                
                // Create 3D bounding box marker
                visualization_msgs::Marker bbox_marker = create3DBoundingBoxMarker(
                    detection, center, bbox_points, marker_id++, cloud_msg->header);
                marker_array.markers.push_back(bbox_marker);
                
                // Create text label marker
                visualization_msgs::Marker text_marker = createTextMarker(
                    detection, center, distance, marker_id++, cloud_msg->header);
                marker_array.markers.push_back(text_marker);
            } else {
                // Even if no bbox points, we still have a valid distance estimate
                // Use distance to estimate 3D position (assuming it's along camera Z-axis)
                det_with_dist.object_center_3d.x = 0.0;  // Rough estimate
                det_with_dist.object_center_3d.y = 0.0;  // Could be improved
                det_with_dist.object_center_3d.z = distance;
            }
            
            // Add this valid detection to the output
            output_msg.detections.push_back(det_with_dist);
        }
        // Note: We completely skip detections without valid distance (no else clause)
    }
    
    auto fusion_end = std::chrono::high_resolution_clock::now();
    output_msg.fusion_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        fusion_end - fusion_start).count() / 1000.0;
    output_msg.detections_with_distance = detections_with_distance;
    
    // Publish results (only if we have valid detections)
    if (!output_msg.detections.empty()) {
        fused_detections_pub_.publish(output_msg);
        
        // Publish 3D bounding boxes
        if (detection_boxes_3d_pub_.getNumSubscribers() > 0 && !marker_array.markers.empty()) {
            detection_boxes_3d_pub_.publish(marker_array);
        }
    }
    
    // Publish debug cloud if enabled
    if (debug_visualization_ && debug_cloud_pub_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 debug_cloud;
        debug_cloud.header = cloud_msg->header;
        
        sensor_msgs::PointCloud2Modifier modifier(debug_cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(filtered_points.size());
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(debug_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(debug_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(debug_cloud, "z");
        
        for (size_t i = 0; i < filtered_points.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = filtered_points[i].x;
            *iter_y = filtered_points[i].y;
            *iter_z = filtered_points[i].z;
        }
        
        debug_cloud_pub_.publish(debug_cloud);
    }
    
    // Enhanced logging - show filtering results
    std::string tf_status = use_tf_transforms_ ? "TF" : "Manual";
    
    if (detections_with_distance > 0) {
        ROS_INFO("Fusion (%s): %d/%d detections matched with LiDAR (%.1f ms, %zu points), published %zu 3D boxes", 
                tf_status.c_str(), detections_with_distance, processed_detections,
                output_msg.fusion_time_ms, filtered_points.size(), marker_array.markers.size()/2);
    } else {
        // Show when no detections were matched (useful for debugging)
        ROS_INFO("Fusion (%s): 0/%d detections matched with LiDAR (%.1f ms, %zu points) - no output published", 
                tf_status.c_str(), processed_detections, output_msg.fusion_time_ms, filtered_points.size());
    }
    
    // Periodically log transform for debugging
    static ros::Time last_transform_log = ros::Time(0);
    if (use_tf_transforms_ && (ros::Time::now() - last_transform_log).toSec() > 10.0) {
        logCurrentTransform();
        last_transform_log = ros::Time::now();
    }
}

visualization_msgs::Marker LidarCameraFusion::createTextMarker(
    const drive_yolo::Detection& detection,
    const cv::Point3f& center_3d,
    double distance,
    int marker_id,
    const std_msgs::Header& header) {
    
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "detection_labels";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = center_3d.x;
    marker.pose.position.y = center_3d.y;
    marker.pose.position.z = center_3d.z + 1.0;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.z = 0.5;
    
    std::stringstream ss;
    ss << detection.class_name << "\n"
       << std::fixed << std::setprecision(1) << (detection.confidence * 100) << "%\n"
       << std::setprecision(1) << distance << "m";
    marker.text = ss.str();
    
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration(0.5);
    return marker;
}

void LidarCameraFusion::setMarkerColor(visualization_msgs::Marker& marker, int class_id, float confidence) {
    switch (class_id) {
        case 0: marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; break; // person - red
        case 1: marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; break; // bicycle - cyan
        case 2: marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; break; // car - blue
        case 3: marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0; break; // motorcycle - magenta
        case 5: marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0; break; // bus - orange
        case 7: marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.0; break; // truck - olive
        default: marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; break; // other - green
    }
    
    float intensity = 0.5f + 0.5f * confidence;
    marker.color.r *= intensity;
    marker.color.g *= intensity;
    marker.color.b *= intensity;
}

void LidarCameraFusion::publishCameraInfo() {
    sensor_msgs::CameraInfo camera_info_msg;
    
    camera_info_msg.header.stamp = ros::Time::now();
    camera_info_msg.header.frame_id = camera_frame_;  // Use the configurable camera frame
    
    int image_width, image_height;
    pnh_.param<int>("image_width", image_width, 3848);
    pnh_.param<int>("image_height", image_height, 2168);
    
    camera_info_msg.height = image_height;
    camera_info_msg.width = image_width;
    camera_info_msg.distortion_model = "plumb_bob";
    
    camera_info_msg.D.resize(dist_coeffs_.total());
    for (int i = 0; i < dist_coeffs_.total(); ++i) {
        camera_info_msg.D[i] = dist_coeffs_.at<double>(i);
    }
    
    for (int i = 0; i < 9; ++i) {
        camera_info_msg.K[i] = camera_matrix_.at<double>(i);
    }
    
    for (int i = 0; i < 9; ++i) {
        camera_info_msg.R[i] = (i % 4 == 0) ? 1.0 : 0.0;
    }
    
    camera_info_msg.P[0] = camera_matrix_.at<double>(0, 0);
    camera_info_msg.P[1] = camera_matrix_.at<double>(0, 1);
    camera_info_msg.P[2] = camera_matrix_.at<double>(0, 2);
    camera_info_msg.P[3] = 0.0;
    camera_info_msg.P[4] = camera_matrix_.at<double>(1, 0);
    camera_info_msg.P[5] = camera_matrix_.at<double>(1, 1);
    camera_info_msg.P[6] = camera_matrix_.at<double>(1, 2);
    camera_info_msg.P[7] = 0.0;
    camera_info_msg.P[8] = camera_matrix_.at<double>(2, 0);
    camera_info_msg.P[9] = camera_matrix_.at<double>(2, 1);
    camera_info_msg.P[10] = camera_matrix_.at<double>(2, 2);
    camera_info_msg.P[11] = 0.0;
    
    camera_info_pub_.publish(camera_info_msg);
}

void LidarCameraFusion::cameraInfoTimerCallback(const ros::TimerEvent&) {
    if (camera_info_pub_.getNumSubscribers() > 0) {
        publishCameraInfo();
    }
}

std::vector<SimplePoint> LidarCameraFusion::parsePointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    std::vector<SimplePoint> points;
    points.reserve(cloud_msg->width * cloud_msg->height);
    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
            points.emplace_back(*iter_x, *iter_y, *iter_z);
        }
    }
    
    return points;
}

std::vector<SimplePoint> LidarCameraFusion::filterPointsByDistance(const std::vector<SimplePoint>& points) {
    std::vector<SimplePoint> filtered_points;
    filtered_points.reserve(points.size());

    for (const auto& point : points) {
        // Basic radial distance filtering
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance < min_distance_ || distance > max_distance_) {
            continue;
        }

        // ROI filtering (ego vehicle coordinate system)
        // Assuming: X = forward, Y = lateral (positive left, negative right), Z = vertical
        if (enable_roi_filtering_) {
            if (point.x < roi_x_min_ || point.x > roi_x_max_) {
                continue;  // Outside forward range
            }
            if (point.y < roi_y_min_ || point.y > roi_y_max_) {
                continue;  // Outside lateral range
            }
        }

        filtered_points.push_back(point);
    }

    return filtered_points;
}

std::vector<cv::Point2f> LidarCameraFusion::projectPointsToImage(const std::vector<SimplePoint>& points) {
    std::vector<cv::Point3f> object_points;
    object_points.reserve(points.size());
    
    for (const auto& point : points) {
        object_points.emplace_back(point.x, point.y, point.z);
    }
    
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, rotation_matrix_, translation_vector_,
                     camera_matrix_, dist_coeffs_, image_points);
    
    return image_points;
}
std::vector<SimplePoint> LidarCameraFusion::getPointsInBoundingBox(
    const drive_yolo::Detection& detection,
    const std::vector<SimplePoint>& points,
    const std::vector<cv::Point2f>& projected_points) {
    
    std::vector<SimplePoint> bbox_points;
    
    float scale_factor = 1.0f;

    float scaled_input_width = detection.width * scale_factor;
    float scaled_input_height = detection.height * scale_factor;


    float x1 = detection.x - scaled_input_width / 2.0f;
    float y1 = detection.y - scaled_input_height / 2.0f;
    float x2 = detection.x + scaled_input_width / 2.0f;
    float y2 = detection.y + scaled_input_height / 2.0f;
    
    // First pass: collect all points in bounding box
    std::vector<SimplePoint> candidate_points;
    int total_in_bbox = 0;
    int filtered_by_ground = 0;
    int filtered_by_distance = 0;

    for (size_t i = 0; i < projected_points.size() && i < points.size(); ++i) {
        const cv::Point2f& proj_point = projected_points[i];

        if (proj_point.x >= x1 && proj_point.x <= x2 &&
            proj_point.y >= y1 && proj_point.y <= y2) {

            total_in_bbox++;
            const SimplePoint& point = points[i];

            // FILTER 1: Ground filtering
            if (enable_ground_filtering_) {
                // Calculate ground level at this point (assuming flat ground for now)
                float ground_level = -lidar_height_above_ground_;  // Ground is below LiDAR
                float point_height_above_ground = point.z - ground_level;

                if (point_height_above_ground < ground_height_threshold_) {
                    filtered_by_ground++;
                    continue;  // Skip ground points
                }
            }

            // FILTER 2: Basic distance filtering (remove points too far away)
            float point_distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (point_distance > max_distance_ || point_distance < min_distance_) {
                filtered_by_distance++;
                continue;
            }

            candidate_points.push_back(point);
        }
    }

    if (candidate_points.empty()) {
        if (total_in_bbox > 0) {
            ROS_WARN("Detection %s: %d points in bbox, but ALL filtered out (ground:%d, distance:%d)",
                     detection.class_name.c_str(), total_in_bbox, filtered_by_ground, filtered_by_distance);
        }
        return bbox_points;
    }

    // FILTER 3: Point clustering and outlier removal
    if (enable_point_clustering_) {
        bbox_points = filterPointsByCluster(candidate_points, detection);
    } else {
        bbox_points = candidate_points;
    }

    ROS_DEBUG("Detection %s: %d in bbox, %zu candidates (-%d ground, -%d dist) -> %zu final points",
             detection.class_name.c_str(), total_in_bbox, candidate_points.size(),
             filtered_by_ground, filtered_by_distance, bbox_points.size());

    return bbox_points;
}

double LidarCameraFusion::estimateObjectDistance(const drive_yolo::Detection& detection,
                                                const std::vector<SimplePoint>& points,
                                                const std::vector<cv::Point2f>& projected_points) {

    std::vector<SimplePoint> bbox_points = getPointsInBoundingBox(detection, points, projected_points);

    if (bbox_points.size() < min_points_for_object_) {
        ROS_WARN("Association FAILED for %s [ID:%d]: Only %zu points (need %d) - bbox[%.0f,%.0f,%.0fx%.0f]",
                 detection.class_name.c_str(), detection.class_id, bbox_points.size(),
                 min_points_for_object_, detection.x, detection.y, detection.width, detection.height);
        return -1.0;
    }

    // Use median distance for robustness against outliers
    double distance = calculateMedianDistance(bbox_points);

    // Sanity check the distance
    if (distance < min_distance_ || distance > max_distance_) {
        ROS_WARN("Association FAILED for %s: Distance %.1f m outside valid range [%.1f, %.1f] with %zu points",
                 detection.class_name.c_str(), distance, min_distance_, max_distance_, bbox_points.size());
        return -1.0;
    }

    ROS_INFO("Association SUCCESS for %s: %.2f m from %zu LiDAR points",
             detection.class_name.c_str(), distance, bbox_points.size());
    return distance;
}

std::vector<SimplePoint> LidarCameraFusion::filterPointsByCluster(
    const std::vector<SimplePoint>& candidate_points,
    const drive_yolo::Detection& detection) {
    
    if (candidate_points.size() < min_points_for_object_) {
        return {};  // Not enough points
    }
    
    // Calculate the centroid of all candidate points
    cv::Point3f centroid(0, 0, 0);
    for (const auto& point : candidate_points) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }
    centroid.x /= candidate_points.size();
    centroid.y /= candidate_points.size();
    centroid.z /= candidate_points.size();
    
    // Find the main cluster (points close to centroid)
    std::vector<SimplePoint> clustered_points;
    std::vector<float> distances_to_centroid;
    
    for (const auto& point : candidate_points) {
        float distance_to_centroid = std::sqrt(
            (point.x - centroid.x) * (point.x - centroid.x) +
            (point.y - centroid.y) * (point.y - centroid.y) +
            (point.z - centroid.z) * (point.z - centroid.z));
        
        distances_to_centroid.push_back(distance_to_centroid);
    }
    
    // Calculate median distance to filter outliers
    std::vector<float> sorted_distances = distances_to_centroid;
    std::sort(sorted_distances.begin(), sorted_distances.end());
    float median_distance = sorted_distances[sorted_distances.size() / 2];
    float distance_threshold = std::min(outlier_distance_threshold_, (double)median_distance * 2.0d);
    
    // Keep points within reasonable distance of the main cluster
    for (size_t i = 0; i < candidate_points.size(); ++i) {
        if (distances_to_centroid[i] <= distance_threshold) {
            const SimplePoint& point = candidate_points[i];
            
            // Additional sanity checks
            float distance_from_center = std::sqrt(point.x * point.x + point.y * point.y);
            if (distance_from_center <= max_point_distance_from_center_) {
                clustered_points.push_back(point);
            }
        }
    }
    
    // Final check: ensure we have enough points and reasonable object dimensions
    if (clustered_points.size() >= min_points_for_object_) {
        cv::Point3f object_size = calculateObjectDimensions(clustered_points);

        if (object_size.x <= max_object_width_ &&
            object_size.y <= max_object_width_ &&
            object_size.z <= max_object_height_) {
            return clustered_points;
        } else {
            ROS_WARN("Cluster filter REJECTED %s: Object too large %.1f x %.1f x %.1f m (max: %.1f x %.1f x %.1f)",
                     detection.class_name.c_str(),
                     object_size.x, object_size.y, object_size.z,
                     max_object_width_, max_object_width_, max_object_height_);
        }
    } else {
        ROS_WARN("Cluster filter REJECTED %s: Only %zu clustered points (need %d)",
                 detection.class_name.c_str(), clustered_points.size(), min_points_for_object_);
    }

    return {};  // Failed filtering
}

// NEW: Calculate object dimensions for validation
cv::Point3f LidarCameraFusion::calculateObjectDimensions(const std::vector<SimplePoint>& points) {
    if (points.empty()) return cv::Point3f(0, 0, 0);
    
    float min_x = points[0].x, max_x = points[0].x;
    float min_y = points[0].y, max_y = points[0].y;
    float min_z = points[0].z, max_z = points[0].z;
    
    for (const auto& point : points) {
        min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z); max_z = std::max(max_z, point.z);
    }
    
    return cv::Point3f(max_x - min_x, max_y - min_y, max_z - min_z);
}

// ENHANCED: Improved 3D bounding box marker creation
visualization_msgs::Marker LidarCameraFusion::create3DBoundingBoxMarker(
    const drive_yolo::Detection& detection,
    const cv::Point3f& center_3d,
    const std::vector<SimplePoint>& bbox_points,
    int marker_id,
    const std_msgs::Header& header) {
    
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "detection_boxes";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = center_3d.x;
    marker.pose.position.y = center_3d.y;
    marker.pose.position.z = center_3d.z;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    if (!bbox_points.empty()) {
        // Calculate smart bounding box dimensions
        cv::Point3f dimensions = calculateObjectDimensions(bbox_points);
        
        // Apply minimum and maximum size constraints
        marker.scale.x = std::max(0.2, std::min(static_cast<double>(dimensions.x), max_object_width_));
        marker.scale.y = std::max(0.2, std::min(static_cast<double>(dimensions.y), max_object_width_));
        marker.scale.z = std::max(0.2, std::min(static_cast<double>(dimensions.z), max_object_height_));
        
        // For very small objects, use class-based defaults
        if (dimensions.x < 0.5 && dimensions.y < 0.5) {
            switch (detection.class_id) {
                case 0:  // person
                    marker.scale.x = 0.6; marker.scale.y = 0.6; marker.scale.z = 1.7; break;
                case 2:  // car
                    marker.scale.x = 4.5; marker.scale.y = 2.0; marker.scale.z = 1.5; break;
                case 1:  // bicycle
                    marker.scale.x = 1.8; marker.scale.y = 0.6; marker.scale.z = 1.2; break;
                case 3:  // motorcycle
                    marker.scale.x = 2.2; marker.scale.y = 0.8; marker.scale.z = 1.3; break;
                case 5:  // bus
                    marker.scale.x = 12.0; marker.scale.y = 2.5; marker.scale.z = 3.0; break;
                case 7:  // truck
                    marker.scale.x = 8.0; marker.scale.y = 2.5; marker.scale.z = 3.5; break;
                default:
                    marker.scale.x = 1.0; marker.scale.y = 1.0; marker.scale.z = 1.0; break;
            }
        }
        
        ROS_DEBUG("3D Box %s: center=[%.1f,%.1f,%.1f] size=[%.1f,%.1f,%.1f] from %zu points",
                 detection.class_name.c_str(),
                 center_3d.x, center_3d.y, center_3d.z,
                 marker.scale.x, marker.scale.y, marker.scale.z,
                 bbox_points.size());
    } else {
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5;  // Default small size
    }
    
    setMarkerColor(marker, detection.class_id, detection.confidence);
    marker.color.a = 0.4;  // Slightly more opaque
    marker.lifetime = ros::Duration(0.5);
    
    return marker;
}

double LidarCameraFusion::calculateAverageDistance(const std::vector<SimplePoint>& points) {
    if (points.empty()) return -1.0;
    
    double total_distance = 0.0;
    for (const auto& point : points) {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        total_distance += distance;
    }
    
    return total_distance / points.size();
}

double LidarCameraFusion::calculateMedianDistance(const std::vector<SimplePoint>& points) {
    if (points.empty()) return -1.0;
    
    std::vector<double> distances;
    distances.reserve(points.size());
    
    for (const auto& point : points) {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        distances.push_back(distance);
    }
    
    std::sort(distances.begin(), distances.end());
    
    size_t mid = distances.size() / 2;
    if (distances.size() % 2 == 0) {
        return (distances[mid - 1] + distances[mid]) / 2.0;
    } else {
        return distances[mid];
    }
}

cv::Point3f LidarCameraFusion::calculateObjectCenter(const std::vector<SimplePoint>& points) {
    if (points.empty()) return cv::Point3f(0, 0, 0);
    float min_dist = FLT_MAX;
    SimplePoint closest;
    for (const auto& p : points) {
        float dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (dist < min_dist) {
            min_dist = dist;
            closest = p;
        }
    }
    
    // Estimate object depth based on class
    float estimated_depth = getEstimatedObjectDepth(2);
    
    // Center = front_face + depth/2
    return cv::Point3f(
        closest.x + estimated_depth/2.0f,
        closest.y,  // Keep lateral position
        closest.z   // Keep height
    );

}

cv::Point3f LidarCameraFusion::calculateObjectCenterEnhanced(
    const std::vector<SimplePoint>& points, 
    const drive_yolo::Detection& detection) {
    
    if (points.empty()) return cv::Point3f(0, 0, 0);
    
    // Method 1: Your proposed front-face approach
    // Find the closest point (front face of the object)
    float min_distance = std::numeric_limits<float>::max();
    SimplePoint closest_point;
    
    for (const auto& point : points) {
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance < min_distance) {
            min_distance = distance;
            closest_point = point;
        }
    }
    
    // Estimate object dimensions from detection class
    float estimated_depth = getEstimatedObjectDepth(detection.class_id);
    
    // Calculate center as: front_face + depth/2
    cv::Point3f enhanced_center;
    enhanced_center.x = closest_point.x + estimated_depth / 2.0f;
    enhanced_center.y = closest_point.y;  // Keep Y from closest point
    enhanced_center.z = closest_point.z;  // Keep Z from closest point
    
    // Method 2: Alternative - Use clustering to ignore outliers
    cv::Point3f clustered_center = calculateCenterWithClustering(points);
    
    // Method 3: Weighted approach - closer points get more weight
    cv::Point3f weighted_center = calculateWeightedCenter(points);
    
    // Choose best method based on point distribution
    float point_spread = calculatePointSpread(points);
    
    if (point_spread > 5.0f) {  // Points very spread out - use front face
        ROS_DEBUG("Using front-face detection for spread-out points");
        return enhanced_center;
    } else if (point_spread > 2.0f) {  // Moderate spread - use weighted
        ROS_DEBUG("Using weighted center for moderate spread");
        return weighted_center;
    } else {  // Points clustered - use clustering
        ROS_DEBUG("Using clustered center for tight points");
        return clustered_center;
    }
}

bool LidarCameraFusion::isValidPointCloudFormat(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    bool has_x = false, has_y = false, has_z = false;
    
    for (const auto& field : cloud_msg->fields) {
        if (field.name == "x") has_x = true;
        if (field.name == "y") has_y = true;
        if (field.name == "z") has_z = true;
    }
    
    return has_x && has_y && has_z;
}

size_t LidarCameraFusion::getPointCloudSize(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    return cloud_msg->width * cloud_msg->height;
}

cv::Point3f LidarCameraFusion::calculateCenterWithClustering(
    const std::vector<SimplePoint>& points) {
    
    if (points.size() < 3) {
        return calculateObjectCenter(points);  // Fall back to simple average
    }
    
    // Simple clustering: find the main cluster of points
    std::vector<SimplePoint> main_cluster;
    
    // Find densest region using sliding window approach
    std::vector<float> distances;
    for (const auto& point : points) {
        distances.push_back(std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z));
    }
    
    // Sort by distance
    std::sort(distances.begin(), distances.end());
    
    // Find the cluster with most points in a 2m window
    float best_center = distances[0];
    int max_count = 0;
    
    for (size_t i = 0; i < distances.size(); ++i) {
        int count = 0;
        float window_center = distances[i];
        
        for (float dist : distances) {
            if (std::abs(dist - window_center) <= 1.0f) {  // 2m window
                count++;
            }
        }
        
        if (count > max_count) {
            max_count = count;
            best_center = window_center;
        }
    }
    
    // Collect points in the main cluster
    for (const auto& point : points) {
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (std::abs(distance - best_center) <= 1.0f) {
            main_cluster.push_back(point);
        }
    }
    
    // Return center of main cluster
    return calculateObjectCenter(main_cluster);
}

// Method 3: Distance-weighted center (closer points matter more)
cv::Point3f LidarCameraFusion::calculateWeightedCenter(const std::vector<SimplePoint>& points) {
    if (points.empty()) return cv::Point3f(0, 0, 0);
    
    float weighted_x = 0, weighted_y = 0, weighted_z = 0;
    float total_weight = 0;
    
    for (const auto& point : points) {
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        
        // Closer points get exponentially higher weight
        float weight = 1.0f / (1.0f + distance * distance * 0.1f);
        
        weighted_x += point.x * weight;
        weighted_y += point.y * weight;
        weighted_z += point.z * weight;
        total_weight += weight;
    }
    
    if (total_weight > 0) {
        return cv::Point3f(
            weighted_x / total_weight,
            weighted_y / total_weight, 
            weighted_z / total_weight
        );
    }
    
    return calculateObjectCenter(points);  // Fallback
}

// Helper: Calculate how spread out the points are
float LidarCameraFusion::calculatePointSpread(const std::vector<SimplePoint>& points) {
    if (points.size() < 2) return 0.0f;
    
    // Calculate distances of all points
    std::vector<float> distances;
    for (const auto& point : points) {
        distances.push_back(std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z));
    }
    
    std::sort(distances.begin(), distances.end());
    
    // Return difference between furthest and closest
    return distances.back() - distances.front();
}
float LidarCameraFusion::getEstimatedObjectDepth(int class_id) {
    switch (class_id) {
        case 0:  return 0.6f;  // person - 60cm depth
        case 1:  return 1.8f;  // bicycle - 1.8m length
        case 2:  return 4.5f;  // car - 4.5m length
        case 3:  return 2.2f;  // motorcycle - 2.2m length
        case 5:  return 12.0f; // bus - 12m length
        case 7:  return 8.0f;  // truck - 8m length
        default: return 2.0f;  // default - 2m
    }
}