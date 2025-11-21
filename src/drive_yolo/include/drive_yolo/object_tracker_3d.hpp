#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <memory>
#include <deque>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "drive_yolo/DetectionsWithDistance.h"
#include "drive_yolo/DetectionWithDistance.h"

struct TrackedObject3D {
    int id;
    int class_id;
    std::string class_name;
    
    // 3D Position and motion
    cv::Point3f position_3d;
    cv::Point3f velocity_3d;
    cv::Point3f acceleration_3d;
    float speed_3d;
    
    // 2D position for association (MISSING in your header!)
    cv::Point2f position_2d;             // Current 2D center (x, y) in pixels
    cv::Size2f size_2d;                  // Current 2D bounding box size
    
    // Timing information
    ros::Time last_update_time;
    ros::Time first_seen_time;
    double expected_dt_;
    double actual_dt_;
    
    // MISSING: These were using different names!
    std::chrono::high_resolution_clock::time_point last_seen;    // For compatibility
    std::chrono::high_resolution_clock::time_point first_seen;   // For compatibility
    
    // Quality metrics
    float confidence;
    float distance_confidence;
    int lidar_points_count;
    int consecutive_misses;
    int total_detections;
    
    // Better history with precise timing
    struct TimedPosition {
        cv::Point3f position;
        ros::Time timestamp;
        double dt_since_previous;
        
        TimedPosition(const cv::Point3f& pos, const ros::Time& time, double dt = 0.0)
            : position(pos), timestamp(time), dt_since_previous(dt) {}
    };
    std::deque<TimedPosition> position_history;
    
    // Kalman filter
    cv::KalmanFilter kalman_3d;
    bool kalman_initialized;
    
    TrackedObject3D(int obj_id, const drive_yolo::DetectionWithDistance& detection, 
                    double expected_framerate = 10.0);
    
    // Timing-aware updates
    void updateWithTiming(const cv::Point3f& measured_pos, const ros::Time& timestamp);
    void predictWithTiming(const ros::Time& current_time);
    
    // Velocity calculation
    cv::Point3f calculateVelocityFromHistory() const;
    cv::Point3f calculateAccelerationFromHistory() const;
    
    // Handle missing frames
    void handleMissedFrames(const ros::Time& current_time);
    
    // Quality assessment
    bool isHighQualityTrack() const;
    float getTrackStability() const;
    
    // MISSING: Motion prediction method
    cv::Point3f predictPositionAfter(double time_seconds) const {
        cv::Point3f predicted = position_3d;
        
        // Simple linear prediction with current velocity
        predicted.x += velocity_3d.x * time_seconds;
        predicted.y += velocity_3d.y * time_seconds;
        predicted.z += velocity_3d.z * time_seconds;
        
        // Add acceleration component if available
        predicted.x += 0.5f * acceleration_3d.x * time_seconds * time_seconds;
        predicted.y += 0.5f * acceleration_3d.y * time_seconds * time_seconds;
        predicted.z += 0.5f * acceleration_3d.z * time_seconds * time_seconds;
        
        return predicted;
    }
    
private:
    void initializeKalman3D(const cv::Point3f& initial_pos, double dt);
    void updateKalman3D(const cv::Point3f& measured_pos, double dt);
    void addToHistory(const cv::Point3f& pos, const ros::Time& timestamp);
};

class MultiObjectTracker3D {
public:
    MultiObjectTracker3D();
    
    // Set expected framerate for better timing
    void setExpectedFramerate(double camera_fps, double lidar_fps) {
        camera_fps_ = camera_fps;
        lidar_fps_ = lidar_fps;
        expected_fusion_fps_ = std::min(camera_fps, lidar_fps);
        expected_dt_ = 1.0 / expected_fusion_fps_;
        
        ROS_INFO("Tracker configured: Camera=%.1f Hz, LiDAR=%.1f Hz, Expected fusion=%.1f Hz", 
                 camera_fps_, lidar_fps_, expected_fusion_fps_);
    }
    
    // Main tracking function
    void updateTracks(const drive_yolo::DetectionsWithDistance::ConstPtr& detections);
    
    // Get tracked objects
    std::vector<TrackedObject3D> getAllTrackedObjects() const;
    std::vector<TrackedObject3D> getHighQualityTracks() const;
    std::vector<TrackedObject3D> getMovingObjects(float min_speed_ms = 0.5f) const;
    
    // Configuration - MISSING methods!
    void setDistanceThreshold3D(float threshold) { distance_threshold_3d_ = threshold; }
    void setMaxMissedFrames(int max_missed) { max_missed_frames_ = max_missed; }
    void setMinDetectionsForTracking(int min_detections) { min_detections_for_tracking_ = min_detections; }
    void setMinLidarPoints(int min_points) { min_lidar_points_ = min_points; }  // MISSING!
    void setMaxTrackingDistance(float max_dist) { max_tracking_distance_ = max_dist; }  // MISSING!
    
    // Analytics - MISSING methods!
    int getTotalTracks() const { return tracked_objects_.size(); }
    int getActiveTracks() const;  // MISSING implementation!
    float getAverageTrackingDistance() const;  // MISSING implementation!
    double getActualFramerate() const;
    double getAverageTimeDelta() const;

private:
    std::unordered_map<int, std::shared_ptr<TrackedObject3D>> tracked_objects_;
    int next_id_;
    
    // Timing configuration
    double camera_fps_;
    double lidar_fps_;
    double expected_fusion_fps_;
    double expected_dt_;
    
    // Frame timing tracking
    ros::Time last_update_time_;
    std::deque<ros::Time> frame_timestamps_;
    std::deque<double> frame_deltas_;
    
    // Parameters
    float distance_threshold_3d_;
    int max_missed_frames_;
    int min_detections_for_tracking_;
    int min_lidar_points_;      // MISSING in your header!
    float max_tracking_distance_;  // MISSING in your header!
    
    // Timing-aware methods
    void predictAllTracks(const ros::Time& current_time);
    void updateFrameTimingStats(const ros::Time& current_time);
    double calculateActualDT(const ros::Time& current_time);
    
    // Core tracking methods
    std::vector<std::pair<int, int>> associateDetections3D(
        const std::vector<drive_yolo::DetectionWithDistance>& detections);
    void updateTrack3D(TrackedObject3D& track, 
                       const drive_yolo::DetectionWithDistance& detection,
                       const ros::Time& timestamp);
    void createNewTracks(const std::vector<drive_yolo::DetectionWithDistance>& unassigned_detections,
                        const ros::Time& timestamp);
    void removeOldTracks(const ros::Time& current_time);
    
    // Utility methods
    cv::Point3f extractPosition3D(const drive_yolo::DetectionWithDistance& detection) const;
    bool isValidDetection(const drive_yolo::DetectionWithDistance& detection) const;
    float calculateDistance3D(const TrackedObject3D& track, 
                             const drive_yolo::DetectionWithDistance& detection) const;
    
    // Timing utilities
    double toSeconds(const ros::Time& t1, const ros::Time& t2) const {
        return (t2 - t1).toSec();
    }
};