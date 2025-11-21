#include "ros/ros.h"
#include "drive_yolo/object_tracker.hpp"
#include "drive_yolo/Detections.h"
#include "drive_yolo/TrackedObjects.h"
#include "drive_yolo/TrackedObject.h"
#include <chrono>

class TrackingNode {
public:
    TrackingNode() : nh_("~") {
        // Get parameters
        nh_.param<double>("distance_threshold", distance_threshold_, 100.0);
        nh_.param<int>("max_missed_frames", max_missed_frames_, 10);
        nh_.param<int>("min_detections_for_tracking", min_detections_for_tracking_, 3);
        nh_.param<bool>("quiet_mode", quiet_mode_, false);
        
        // Configure tracker
        tracker_.setDistanceThreshold(distance_threshold_);
        tracker_.setMaxMissedFrames(max_missed_frames_);
        tracker_.setMinDetectionsForTracking(min_detections_for_tracking_);
        
        // Setup ROS communication
        detections_sub_ = nh_.subscribe("/detections", 10, &TrackingNode::detectionsCallback, this);
        tracked_objects_pub_ = nh_.advertise<drive_yolo::TrackedObjects>("/tracked_objects", 10);
        
        ROS_INFO("=== OBJECT TRACKER INITIALIZED ===");
        ROS_INFO("Distance threshold: %.1f pixels", distance_threshold_);
        ROS_INFO("Max missed frames: %d", max_missed_frames_);
        ROS_INFO("Min detections for tracking: %d", min_detections_for_tracking_);
        ROS_INFO("Quiet mode: %s", quiet_mode_ ? "ENABLED" : "DISABLED");
        ROS_INFO("Subscribing to: /detections");
        ROS_INFO("Publishing to: /tracked_objects");
        ROS_INFO("================================");
        
        // Performance tracking
        frame_count_ = 0;
        start_time_ = std::chrono::high_resolution_clock::now();
        last_stats_report_ = start_time_;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber detections_sub_;
    ros::Publisher tracked_objects_pub_;
    
    MultiObjectTracker tracker_;
    
    // Parameters
    double distance_threshold_;
    int max_missed_frames_;
    int min_detections_for_tracking_;
    bool quiet_mode_;
    
    // Performance tracking
    int frame_count_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_stats_report_;
    
    void detectionsCallback(const drive_yolo::Detections::ConstPtr& msg) {
        auto callback_start = std::chrono::high_resolution_clock::now();
        
        // Update tracker with new detections
        tracker_.updateTracks(msg);
        
        // Get tracked objects
        auto tracked_objects = tracker_.getTrackedObjects();
        
        // Publish tracked objects
        publishTrackedObjects(tracked_objects, msg->header);
        
        auto callback_end = std::chrono::high_resolution_clock::now();
        double processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
            callback_end - callback_start).count() / 1000.0;
        
        frame_count_++;
        
        // Log tracking results (if not in quiet mode)
        if (!quiet_mode_ && !tracked_objects.empty()) {
            ROS_INFO("Frame %d: %.1f ms, %zu tracks", frame_count_, processing_time, tracked_objects.size());
            
            // Show details for up to 3 tracks
            for (size_t i = 0; i < tracked_objects.size() && i < 3; ++i) {
                const auto& track = tracked_objects[i];
                ROS_INFO("  Track %d (%s): pos=[%.1f,%.1f] vel=[%.1f,%.1f] speed=%.1f px/s conf=%.1f%% det=%d", 
                         track.id, track.class_name.c_str(),
                         track.position.x, track.position.y,
                         track.velocity.x, track.velocity.y, track.speed,
                         track.confidence * 100, track.total_detections);
            }
            if (tracked_objects.size() > 3) {
                ROS_INFO("  ... and %zu more tracks", tracked_objects.size() - 3);
            }
        }
        
        // Report stats every 15 seconds
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_report = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - last_stats_report_).count();
        
        if (time_since_report >= 15) {
            auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - start_time_).count();
            double fps = static_cast<double>(frame_count_) / total_time;
            
            ROS_INFO("=== TRACKING STATS ===");
            ROS_INFO("Frames processed: %d (%.1f Hz)", frame_count_, fps);
            ROS_INFO("Active tracks: %zu", tracked_objects.size());
            ROS_INFO("Processing time: %.1f ms avg", processing_time);
            ROS_INFO("=====================");
            
            last_stats_report_ = current_time;
        }
    }
    
    void publishTrackedObjects(const std::vector<TrackedObject>& tracked_objects, 
                              const std_msgs::Header& original_header) {
        drive_yolo::TrackedObjects msg;
        msg.header = original_header;
        msg.total_tracks = tracked_objects.size();
        
        for (const auto& track : tracked_objects) {
            drive_yolo::TrackedObject track_msg;
            track_msg.id = track.id;
            track_msg.class_id = track.class_id;
            track_msg.class_name = track.class_name;
            track_msg.x = track.position.x;
            track_msg.y = track.position.y;
            track_msg.width = track.size.width;
            track_msg.height = track.size.height;
            track_msg.confidence = track.confidence;
            track_msg.velocity_x = track.velocity.x;
            track_msg.velocity_y = track.velocity.y;
            track_msg.speed = track.speed;
            
            // Calculate track duration
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                track.last_seen - track.first_seen).count() / 1000.0;
            track_msg.track_duration = duration;
            track_msg.total_detections = track.total_detections;
            
            msg.objects.push_back(track_msg);
        }
        
        tracked_objects_pub_.publish(msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_node");
    
    try {
        TrackingNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Tracking node failed: %s", e.what());
        return -1;
    }
    
    return 0;
}