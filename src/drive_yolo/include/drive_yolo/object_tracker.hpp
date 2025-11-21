#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <memory>
#include "drive_yolo/Detection.h"
#include "drive_yolo/Detections.h"

struct TrackedObject {
    int id;                              // Unique tracking ID
    int class_id;                        // Object class ID
    std::string class_name;              // Object class name
    cv::Point2f position;                // Current center position (x, y)
    cv::Point2f velocity;                // Velocity in pixels/second (vx, vy)
    float speed;                         // Speed magnitude in pixels/second
    cv::Size2f size;                     // Current bounding box size (width, height)
    float confidence;                    // Current detection confidence
    
    // Tracking state
    std::chrono::high_resolution_clock::time_point last_seen;
    std::chrono::high_resolution_clock::time_point first_seen;
    int consecutive_misses;              // Number of consecutive frames without detection
    int total_detections;                // Total number of times detected
    
    // Kalman filter state
    cv::KalmanFilter kalman;
    bool kalman_initialized;
    
    // History for smoothing
    std::vector<cv::Point2f> position_history;
    std::vector<std::chrono::high_resolution_clock::time_point> time_history;
    
    TrackedObject(int obj_id, const drive_yolo::Detection& detection);
    void updateKalman(const cv::Point2f& measured_pos, double dt);
    void predictKalman(double dt);
    cv::Point2f getSmoothedVelocity() const;
    float getSmoothedSpeed() const;
    void addToHistory(const cv::Point2f& pos, const std::chrono::high_resolution_clock::time_point& time);
};

class MultiObjectTracker {
public:
    MultiObjectTracker();
    ~MultiObjectTracker() = default;
    
    // Main tracking function
    void updateTracks(const drive_yolo::Detections::ConstPtr& detections);
    
    // Get current tracked objects
    std::vector<TrackedObject> getTrackedObjects() const;
    
    // Configuration
    void setMaxMissedFrames(int max_missed) { max_missed_frames_ = max_missed; }
    void setDistanceThreshold(float threshold) { distance_threshold_ = threshold; }
    void setMinDetectionsForTracking(int min_detections) { min_detections_for_tracking_ = min_detections; }
    void setHistorySize(int size) { history_size_ = size; }
    
private:
    std::unordered_map<int, std::shared_ptr<TrackedObject>> tracked_objects_;
    int next_id_;
    
    // Configuration parameters
    int max_missed_frames_;              // Maximum frames to keep track without detection
    float distance_threshold_;           // Maximum distance for association (pixels)
    int min_detections_for_tracking_;    // Minimum detections before publishing track
    int history_size_;                   // Number of positions to keep in history
    
    // Internal methods
    std::vector<std::pair<int, int>> associateDetections(const std::vector<drive_yolo::Detection>& detections);
    float calculateDistance(const TrackedObject& track, const drive_yolo::Detection& detection) const;
    void updateTrack(TrackedObject& track, const drive_yolo::Detection& detection);
    void predictTracks(double dt);
    void removeOldTracks();
    void createNewTracks(const std::vector<drive_yolo::Detection>& unassigned_detections);
    cv::KalmanFilter createKalmanFilter() const;
};

// ROS message for tracked objects
// This would go in a new .msg file: TrackedObject.msg
/*
int32 id
int32 class_id  
string class_name
float32 x
float32 y
float32 width
float32 height
float32 confidence
float32 velocity_x      # pixels/second
float32 velocity_y      # pixels/second  
float32 speed           # magnitude in pixels/second
float32 track_duration  # seconds since first detection
int32 total_detections  # number of detections
*/

// ROS message for multiple tracked objects  
// This would go in a new .msg file: TrackedObjects.msg
/*
std_msgs/Header header
TrackedObject[] objects
int32 total_tracks
*/