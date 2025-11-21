#include <ros/ros.h>
#include "drive_yolo/object_tracker_3d.hpp"
#include "drive_yolo/DetectionsWithDistance.h"
#include "drive_yolo/TrackedObjects3D.h"    // Need to create this message
#include "drive_yolo/TrackedObject3D.h"     // Need to create this message
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>
#include <chrono>

class Tracking3DNode {
public:
    Tracking3DNode() : nh_("~") {
        // Get parameters
        nh_.param<double>("distance_threshold_3d", distance_threshold_3d_, 2.0);  // 2 meters
        nh_.param<int>("max_missed_frames", max_missed_frames_, 8);
        nh_.param<int>("min_detections_for_tracking", min_detections_for_tracking_, 3);
        nh_.param<int>("min_lidar_points", min_lidar_points_, 5);
        nh_.param<float>("min_speed_threshold", min_speed_threshold_, 0.1);  // 0.1 m/s
        nh_.param<bool>("quiet_mode", quiet_mode_, false);
        nh_.param<bool>("publish_all_tracks", publish_all_tracks_, true);
        nh_.param<bool>("publish_high_quality_only", publish_high_quality_only_, false);
        nh_.param<float>("max_tracking_distance", max_tracking_distance_, 30.0);  // 30 meters
        
        // Configure tracker
        tracker_.setDistanceThreshold3D(distance_threshold_3d_);
        tracker_.setMaxMissedFrames(max_missed_frames_);
        tracker_.setMinDetectionsForTracking(min_detections_for_tracking_);
        tracker_.setMinLidarPoints(min_lidar_points_);
        tracker_.setMaxTrackingDistance(max_tracking_distance_);
        
        // Setup ROS communication
        detections_sub_ = nh_.subscribe("/detections_with_distance", 10, 
                                       &Tracking3DNode::detectionsCallback, this);
        
        tracked_objects_pub_ = nh_.advertise<drive_yolo::TrackedObjects3D>("/tracked_objects_3d", 10);
        
        // Optional publishers for filtered results
        high_quality_tracks_pub_ = nh_.advertise<drive_yolo::TrackedObjects3D>("/high_quality_tracks_3d", 10);
        moving_objects_pub_ = nh_.advertise<drive_yolo::TrackedObjects3D>("/moving_objects_3d", 10);
        
        ROS_INFO("=== 3D OBJECT TRACKER INITIALIZED ===");
        ROS_INFO("Distance threshold (3D): %.1f meters", distance_threshold_3d_);
        ROS_INFO("Max missed frames: %d", max_missed_frames_);
        ROS_INFO("Min detections for tracking: %d", min_detections_for_tracking_);
        ROS_INFO("Min LiDAR points: %d", min_lidar_points_);
        ROS_INFO("Min speed threshold: %.2f m/s", min_speed_threshold_);
        ROS_INFO("Max tracking distance: %.1f meters", max_tracking_distance_);
        ROS_INFO("Quiet mode: %s", quiet_mode_ ? "ENABLED" : "DISABLED");
        ROS_INFO("Publish all tracks: %s", publish_all_tracks_ ? "YES" : "NO");
        ROS_INFO("High quality only: %s", publish_high_quality_only_ ? "YES" : "NO");
        ROS_INFO("Subscribing to: /detections_with_distance");
        ROS_INFO("Publishing to: /tracked_objects_3d, /high_quality_tracks_3d, /moving_objects_3d");
        ROS_INFO("====================================");
        
        // Performance tracking
        frame_count_ = 0;
        start_time_ = std::chrono::high_resolution_clock::now();
        last_stats_report_ = start_time_;
        total_processing_time_ = 0.0;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber detections_sub_;
    ros::Publisher tracked_objects_pub_;
    ros::Publisher high_quality_tracks_pub_;
    ros::Publisher moving_objects_pub_;
    
    MultiObjectTracker3D tracker_;
    
    // Parameters
    double distance_threshold_3d_;
    int max_missed_frames_;
    int min_detections_for_tracking_;
    int min_lidar_points_;
    float min_speed_threshold_;
    bool quiet_mode_;
    bool publish_all_tracks_;
    bool publish_high_quality_only_;
    float max_tracking_distance_;
    
    // Performance tracking
    int frame_count_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_stats_report_;
    double total_processing_time_;
    
    void detectionsCallback(const drive_yolo::DetectionsWithDistance::ConstPtr& msg) {
        auto callback_start = std::chrono::high_resolution_clock::now();
        
        // Update tracker with new detections
        tracker_.updateTracks(msg);
        
        // Get different types of tracks
        auto all_tracks = tracker_.getAllTrackedObjects();
        auto high_quality_tracks = tracker_.getHighQualityTracks();
        auto moving_objects = tracker_.getMovingObjects(min_speed_threshold_);
        
        auto callback_end = std::chrono::high_resolution_clock::now();
        double processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
            callback_end - callback_start).count() / 1000.0;
        
        frame_count_++;
        total_processing_time_ += processing_time;
        
        // Publish results
        if (publish_all_tracks_) {
            publishTrackedObjects(all_tracks, msg->header, processing_time, tracked_objects_pub_);
        }
        
        if (publish_high_quality_only_) {
            publishTrackedObjects(high_quality_tracks, msg->header, processing_time, tracked_objects_pub_);
        }
        
        // Always publish filtered results
        publishTrackedObjects(high_quality_tracks, msg->header, processing_time, high_quality_tracks_pub_);
        publishTrackedObjects(moving_objects, msg->header, processing_time, moving_objects_pub_);
        
        // Log tracking results (if not in quiet mode)
        if (!quiet_mode_ && !all_tracks.empty()) {
            ROS_INFO("Frame %d: %.1f ms, %zu total tracks (%zu HQ, %zu moving)", 
                    frame_count_, processing_time, all_tracks.size(), 
                    high_quality_tracks.size(), moving_objects.size());
            
            // Show details for up to 3 high-quality tracks
            for (size_t i = 0; i < high_quality_tracks.size() && i < 3; ++i) {
                const auto& track = high_quality_tracks[i];
                ROS_INFO("  Track %d (%s): pos=[%.2f,%.2f,%.2f]m vel=[%.2f,%.2f,%.2f]m/s speed=%.2fm/s conf=%.1f%% pts=%d", 
                         track.id, track.class_name.c_str(),
                         track.position_3d.x, track.position_3d.y, track.position_3d.z,
                         track.velocity_3d.x, track.velocity_3d.y, track.velocity_3d.z, 
                         track.speed_3d, track.confidence * 100, track.lidar_points_count);
            }
            if (high_quality_tracks.size() > 3) {
                ROS_INFO("  ... and %zu more high-quality tracks", high_quality_tracks.size() - 3);
            }
        }
        
        // Report stats every 20 seconds
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_report = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - last_stats_report_).count();
        
        if (time_since_report >= 20) {
            auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - start_time_).count();
            double fps = static_cast<double>(frame_count_) / total_time;
            double avg_processing = total_processing_time_ / frame_count_;
            
            ROS_INFO("=== 3D TRACKING STATS ===");
            ROS_INFO("Frames processed: %d (%.1f Hz)", frame_count_, fps);
            ROS_INFO("Active tracks: %d (total: %d)", tracker_.getActiveTracks(), tracker_.getTotalTracks());
            ROS_INFO("High quality tracks: %zu", high_quality_tracks.size());
            ROS_INFO("Moving objects: %zu", moving_objects.size());
            ROS_INFO("Avg processing time: %.1f ms", avg_processing);
            ROS_INFO("Avg tracking distance: %.1f m", tracker_.getAverageTrackingDistance());
            ROS_INFO("========================");
            
            last_stats_report_ = current_time;
        }
    }
    
    void publishTrackedObjects(const std::vector<TrackedObject3D>& tracked_objects, 
                              const std_msgs::Header& original_header,
                              double processing_time_ms,
                              ros::Publisher& publisher) {
        drive_yolo::TrackedObjects3D msg;
        msg.header = original_header;
        msg.total_tracks = tracked_objects.size();
        msg.processing_time_ms = processing_time_ms;
        
        int high_quality_count = 0;
        int moving_count = 0;
        
        for (const auto& track : tracked_objects) {
            drive_yolo::TrackedObject3D track_msg;
            
            // Basic info
            track_msg.id = track.id;
            track_msg.class_id = track.class_id;
            track_msg.class_name = track.class_name;
            
            // 3D position and motion
            track_msg.position_3d.x = track.position_3d.x;
            track_msg.position_3d.y = track.position_3d.y;
            track_msg.position_3d.z = track.position_3d.z;
            
            track_msg.velocity_3d.x = track.velocity_3d.x;
            track_msg.velocity_3d.y = track.velocity_3d.y;
            track_msg.velocity_3d.z = track.velocity_3d.z;
            
            track_msg.acceleration_3d.x = track.acceleration_3d.x;
            track_msg.acceleration_3d.y = track.acceleration_3d.y;
            track_msg.acceleration_3d.z = track.acceleration_3d.z;
            
            track_msg.speed_3d = track.speed_3d;
            
            // 2D bounding box (for visualization)
            track_msg.x = track.position_2d.x;
            track_msg.y = track.position_2d.y;
            track_msg.width = track.size_2d.width;
            track_msg.height = track.size_2d.height;
            
            // Quality metrics
            track_msg.confidence = track.confidence;
            track_msg.distance_confidence = track.distance_confidence;
            track_msg.lidar_points_count = track.lidar_points_count;
            track_msg.track_stability = track.getTrackStability();
            
            // Tracking info
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                track.last_seen - track.first_seen).count() / 1000.0;
            track_msg.track_duration = duration;
            track_msg.total_detections = track.total_detections;
            track_msg.is_moving = track.speed_3d > min_speed_threshold_;
            
            // Prediction
            cv::Point3f predicted = track.predictPositionAfter(1.0);  // 1 second ahead
            track_msg.predicted_position_1s.x = predicted.x;
            track_msg.predicted_position_1s.y = predicted.y;
            track_msg.predicted_position_1s.z = predicted.z;
            
            msg.objects.push_back(track_msg);
            
            // Count categories
            if (track.isHighQualityTrack()) high_quality_count++;
            if (track.speed_3d > min_speed_threshold_) moving_count++;
        }
        
        msg.high_quality_tracks = high_quality_count;
        msg.moving_objects = moving_count;
        
        publisher.publish(msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_3d_node");
    
    try {
        Tracking3DNode node;
        
        ROS_INFO("3D Object Tracking Node running... Press Ctrl+C to stop");
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("3D Tracking node failed: %s", e.what());
        return -1;
    }
    
    ROS_INFO("3D Object Tracking Node shutting down");
    return 0;
}