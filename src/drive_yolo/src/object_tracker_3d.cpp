#include "drive_yolo/object_tracker_3d.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// ===== TrackedObject3D Implementation =====

TrackedObject3D::TrackedObject3D(int obj_id, const drive_yolo::DetectionWithDistance& detection, 
                                 double expected_framerate)
    : id(obj_id)
    , class_id(detection.detection.class_id)
    , class_name(detection.detection.class_name)
    , velocity_3d(0.0f, 0.0f, 0.0f)
    , acceleration_3d(0.0f, 0.0f, 0.0f)
    , speed_3d(0.0f)
    , confidence(detection.detection.confidence)
    , distance_confidence(detection.distance_confidence)
    , lidar_points_count(detection.lidar_points_count)
    , consecutive_misses(0)
    , total_detections(1)
    , kalman_initialized(false)
    , expected_dt_(1.0 / expected_framerate)
{
    // Extract 3D position
    if (detection.distance > 0 && detection.lidar_points_count > 0) {
        position_3d = cv::Point3f(detection.object_center_3d.x, 
                                 detection.object_center_3d.y, 
                                 detection.object_center_3d.z);
    } else {
        position_3d = cv::Point3f(0, 0, static_cast<float>(detection.distance));
    }
    
    // Initialize timing
    last_update_time = ros::Time::now();
    first_seen_time = last_update_time;
    actual_dt_ = expected_dt_;
    
    // Initialize history
    //position_history.reserve(20);
    addToHistory(position_3d, last_update_time);
    
    ROS_DEBUG("Created track %d at [%.2f, %.2f, %.2f] with expected dt=%.3fs", 
             id, position_3d.x, position_3d.y, position_3d.z, expected_dt_);
}

void TrackedObject3D::updateWithTiming(const cv::Point3f& measured_pos, const ros::Time& timestamp) {
    // Calculate actual time delta
    actual_dt_ = (timestamp - last_update_time).toSec();
    
    // Detect timing issues
    if (actual_dt_ < 0.001) {
        ROS_WARN("Track %d: Very small dt=%.6fs, skipping update", id, actual_dt_);
        return;
    }
    
    if (actual_dt_ > expected_dt_ * 3.0) {
        ROS_WARN("Track %d: Large dt=%.3fs (expected %.3fs), may affect accuracy", 
                id, actual_dt_, expected_dt_);
    }
    
    // Update Kalman filter with actual dt
    updateKalman3D(measured_pos, actual_dt_);
    
    // Update timing
    last_update_time = timestamp;
    consecutive_misses = 0;
    total_detections++;
    
    // Add to history with timing
    addToHistory(measured_pos, timestamp);
    
    // Calculate velocities from history (more accurate than Kalman for low framerates)
    velocity_3d = calculateVelocityFromHistory();
    acceleration_3d = calculateAccelerationFromHistory();
    speed_3d = std::sqrt(velocity_3d.x * velocity_3d.x + 
                        velocity_3d.y * velocity_3d.y + 
                        velocity_3d.z * velocity_3d.z);
    
    ROS_DEBUG("Track %d updated: dt=%.3fs, speed=%.2fm/s (%.1fkm/h)", 
             id, actual_dt_, speed_3d, speed_3d * 3.6);
}

void TrackedObject3D::predictWithTiming(const ros::Time& current_time) {
    if (!kalman_initialized) return;
    
    double dt = (current_time - last_update_time).toSec();
    if (dt < 0.001) return;  // Skip very small time steps
    
    // Update transition matrix
    kalman_3d.transitionMatrix.at<float>(0, 3) = dt;
    kalman_3d.transitionMatrix.at<float>(1, 4) = dt;
    kalman_3d.transitionMatrix.at<float>(2, 5) = dt;
    
    // Predict
    cv::Mat prediction = kalman_3d.predict();
    position_3d.x = prediction.at<float>(0, 0);
    position_3d.y = prediction.at<float>(1, 0);
    position_3d.z = prediction.at<float>(2, 0);
    
    // Update miss count
    consecutive_misses++;
}

void TrackedObject3D::handleMissedFrames(const ros::Time& current_time) {
    double time_since_last = (current_time - last_update_time).toSec();
    int estimated_missed_frames = static_cast<int>(time_since_last / expected_dt_);
    
    if (estimated_missed_frames > consecutive_misses) {
        consecutive_misses = estimated_missed_frames;
        ROS_DEBUG("Track %d: Detected %d missed frames (%.3fs gap)", 
                 id, estimated_missed_frames, time_since_last);
    }
}

cv::Point3f TrackedObject3D::calculateVelocityFromHistory() const {
    if (position_history.size() < 2) return cv::Point3f(0, 0, 0);
    
    // Use multiple points for more stable velocity estimation
    size_t points_to_use = std::min(position_history.size(), static_cast<size_t>(5));
    if (points_to_use < 2) return cv::Point3f(0, 0, 0);
    
    // Linear regression over recent history
    cv::Point3f velocity_sum(0, 0, 0);
    double time_sum = 0;
    int valid_samples = 0;
    
    for (size_t i = position_history.size() - points_to_use; 
         i < position_history.size() - 1; ++i) {
        
        const auto& p1 = position_history[i];
        const auto& p2 = position_history[i + 1];
        
        double dt = (p2.timestamp - p1.timestamp).toSec();
        
        if (dt > 0.001) {  // Valid time delta
            cv::Point3f vel;
            vel.x = (p2.position.x - p1.position.x) / dt;
            vel.y = (p2.position.y - p1.position.y) / dt;
            vel.z = (p2.position.z - p1.position.z) / dt;
            
            // Weight recent samples more heavily
            double weight = 1.0 + (i - (position_history.size() - points_to_use)) * 0.2;
            
            velocity_sum.x += vel.x * weight;
            velocity_sum.y += vel.y * weight;
            velocity_sum.z += vel.z * weight;
            time_sum += weight;
            valid_samples++;
        }
    }
    
    if (valid_samples > 0 && time_sum > 0) {
        cv::Point3f avg_velocity;
        avg_velocity.x = velocity_sum.x / time_sum;
        avg_velocity.y = velocity_sum.y / time_sum;
        avg_velocity.z = velocity_sum.z / time_sum;
        return avg_velocity;
    }
    
    return cv::Point3f(0, 0, 0);
}

cv::Point3f TrackedObject3D::calculateAccelerationFromHistory() const {
    if (position_history.size() < 3) return cv::Point3f(0, 0, 0);
    
    // Use last 3 points to calculate acceleration
    const auto& p1 = position_history[position_history.size() - 3];
    const auto& p2 = position_history[position_history.size() - 2];
    const auto& p3 = position_history[position_history.size() - 1];
    
    double dt1 = (p2.timestamp - p1.timestamp).toSec();
    double dt2 = (p3.timestamp - p2.timestamp).toSec();
    
    if (dt1 > 0.001 && dt2 > 0.001) {
        // Calculate velocities
        cv::Point3f v1((p2.position.x - p1.position.x) / dt1,
                      (p2.position.y - p1.position.y) / dt1,
                      (p2.position.z - p1.position.z) / dt1);
        
        cv::Point3f v2((p3.position.x - p2.position.x) / dt2,
                      (p3.position.y - p2.position.y) / dt2,
                      (p3.position.z - p2.position.z) / dt2);
        
        // Calculate acceleration
        cv::Point3f accel((v2.x - v1.x) / dt2,
                         (v2.y - v1.y) / dt2,
                         (v2.z - v1.z) / dt2);
        
        return accel;
    }
    
    return cv::Point3f(0, 0, 0);
}

bool TrackedObject3D::isHighQualityTrack() const {
    return total_detections >= 3 &&
           consecutive_misses <= 2 && 
           lidar_points_count >= 5 &&
           distance_confidence > 0.5f &&
           confidence > 0.3f &&
           position_history.size() >= 3;
}

float TrackedObject3D::getTrackStability() const {
    float stability = 0.0f;
    
    // Factor 1: Detection consistency
    if (total_detections > 0) {
        stability += 0.3f * std::min(1.0f, static_cast<float>(total_detections) / 10.0f);
    }
    
    // Factor 2: Recent detections (low miss rate)
    stability += 0.2f * std::max(0.0f, 1.0f - static_cast<float>(consecutive_misses) / 5.0f);
    
    // Factor 3: LiDAR quality
    stability += 0.2f * distance_confidence;
    stability += 0.1f * std::min(1.0f, static_cast<float>(lidar_points_count) / 20.0f);
    
    // Factor 4: Confidence
    stability += 0.1f * confidence;
    
    // Factor 5: Position history consistency
    if (position_history.size() >= 3) {
        stability += 0.1f;  // Bonus for having enough history
    }
    
    return std::min(1.0f, stability);
}

void TrackedObject3D::addToHistory(const cv::Point3f& pos, const ros::Time& timestamp) {
    double dt = 0.0;
    if (!position_history.empty()) {
        dt = (timestamp - position_history.back().timestamp).toSec();
    }
    
    position_history.emplace_back(pos, timestamp, dt);
    
    // Keep reasonable history size
    while (position_history.size() > 20) {
        position_history.pop_front();
    }
}

void TrackedObject3D::updateKalman3D(const cv::Point3f& measured_pos, double dt) {
    if (!kalman_initialized) {
        initializeKalman3D(measured_pos, dt);
        return;
    }
    
    // Update transition matrix with actual dt
    kalman_3d.transitionMatrix.at<float>(0, 3) = dt;
    kalman_3d.transitionMatrix.at<float>(1, 4) = dt;
    kalman_3d.transitionMatrix.at<float>(2, 5) = dt;
    
    // Predict and correct
    cv::Mat prediction = kalman_3d.predict();
    cv::Mat measurement = (cv::Mat_<float>(3, 1) << measured_pos.x, measured_pos.y, measured_pos.z);
    kalman_3d.correct(measurement);
    
    // Update position from Kalman (smoothed)
    position_3d.x = kalman_3d.statePost.at<float>(0, 0);
    position_3d.y = kalman_3d.statePost.at<float>(1, 0);
    position_3d.z = kalman_3d.statePost.at<float>(2, 0);
}

void TrackedObject3D::initializeKalman3D(const cv::Point3f& initial_pos, double dt) {
    kalman_3d.init(6, 3, 0);  // [x, y, z, vx, vy, vz]
    
    // State transition matrix
    kalman_3d.transitionMatrix = (cv::Mat_<float>(6, 6) << 
        1, 0, 0, dt, 0,  0,
        0, 1, 0, 0,  dt, 0,
        0, 0, 1, 0,  0,  dt,
        0, 0, 0, 1,  0,  0,
        0, 0, 0, 0,  1,  0,
        0, 0, 0, 0,  0,  1);
    
    // Measurement matrix
    kalman_3d.measurementMatrix = (cv::Mat_<float>(3, 6) << 
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0);
    
    // Process noise (tuned for vehicle speeds)
    kalman_3d.processNoiseCov = cv::Mat::eye(6, 6, CV_32F) * 0.2f;
    
    // Measurement noise (LiDAR accuracy ~10cm)
    kalman_3d.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 0.1f;
    
    // Initial state
    kalman_3d.statePre = (cv::Mat_<float>(6, 1) << 
        initial_pos.x, initial_pos.y, initial_pos.z, 0, 0, 0);
    kalman_3d.statePost = kalman_3d.statePre.clone();
    
    kalman_3d.errorCovPost = cv::Mat::eye(6, 6, CV_32F);
    
    kalman_initialized = true;
    
    ROS_DEBUG("Kalman initialized for track %d with dt=%.3fs", id, dt);
}

// ===== MultiObjectTracker3D Implementation =====

MultiObjectTracker3D::MultiObjectTracker3D() 
    : next_id_(1)
    , camera_fps_(30.0)
    , lidar_fps_(10.0) 
    , expected_fusion_fps_(10.0)
    , expected_dt_(0.1)
    , distance_threshold_3d_(2.0f)
    , max_missed_frames_(8)
    , min_detections_for_tracking_(3)
    , min_lidar_points_(5)
    , max_tracking_distance_(30.0f)
{
    last_update_time_ = ros::Time::now();
}

void MultiObjectTracker3D::updateTracks(const drive_yolo::DetectionsWithDistance::ConstPtr& detections) {
    ros::Time current_time = detections->header.stamp;
    
    // Update timing statistics
    updateFrameTimingStats(current_time);
    
    // Predict all existing tracks
    predictAllTracks(current_time);
    
    // Filter valid detections
    std::vector<drive_yolo::DetectionWithDistance> valid_detections;
    for (const auto& detection : detections->detections) {
        if (isValidDetection(detection)) {
            valid_detections.push_back(detection);
        }
    }
    
    // Associate detections with existing tracks
    auto associations = associateDetections3D(valid_detections);
    
    // Update associated tracks
    std::vector<bool> detection_used(valid_detections.size(), false);
    for (const auto& association_pair : associations) {
        int track_id = association_pair.first;
        int detection_idx = association_pair.second;

        auto track_it = tracked_objects_.find(track_id);
        if (track_it != tracked_objects_.end()) {
            updateTrack3D(*track_it->second, valid_detections[detection_idx], current_time);
            detection_used[detection_idx] = true;
        }
    }
    
    // Create new tracks for unassigned detections
    std::vector<drive_yolo::DetectionWithDistance> unassigned_detections;
    for (size_t i = 0; i < valid_detections.size(); ++i) {
        if (!detection_used[i]) {
            unassigned_detections.push_back(valid_detections[i]);
        }
    }
    createNewTracks(unassigned_detections, current_time);
    
    // Remove old tracks
    removeOldTracks(current_time);
    
    last_update_time_ = current_time;
}

void MultiObjectTracker3D::predictAllTracks(const ros::Time& current_time) {
    for (auto& [id, track] : tracked_objects_) {
        track->predictWithTiming(current_time);
        track->handleMissedFrames(current_time);
    }
}

void MultiObjectTracker3D::updateFrameTimingStats(const ros::Time& current_time) {
    if (!last_update_time_.isZero()) {
        double dt = (current_time - last_update_time_).toSec();
        frame_deltas_.push_back(dt);
        
        // Keep only recent deltas for statistics
        while (frame_deltas_.size() > 50) {
            frame_deltas_.pop_front();
        }
    }
    
    frame_timestamps_.push_back(current_time);
    while (frame_timestamps_.size() > 50) {
        frame_timestamps_.pop_front();
    }
}

double MultiObjectTracker3D::getActualFramerate() const {
    if (frame_timestamps_.size() < 2) return 0.0;
    
    double total_time = (frame_timestamps_.back() - frame_timestamps_.front()).toSec();
    return static_cast<double>(frame_timestamps_.size() - 1) / total_time;
}

double MultiObjectTracker3D::getAverageTimeDelta() const {
    if (frame_deltas_.empty()) return expected_dt_;
    
    double sum = std::accumulate(frame_deltas_.begin(), frame_deltas_.end(), 0.0);
    return sum / frame_deltas_.size();
}

void MultiObjectTracker3D::updateTrack3D(TrackedObject3D& track, 
                                        const drive_yolo::DetectionWithDistance& detection,
                                        const ros::Time& timestamp) {
    // Extract 3D position
    cv::Point3f measured_pos = extractPosition3D(detection);
    
    // Update with timing awareness
    track.updateWithTiming(measured_pos, timestamp);
    
    // Update other properties (FIXED: Added 2D position and size updates)
    track.position_2d = cv::Point2f(detection.detection.x, detection.detection.y);
    track.size_2d = cv::Size2f(detection.detection.width, detection.detection.height);
    track.confidence = detection.detection.confidence;
    track.distance_confidence = detection.distance_confidence;
    track.lidar_points_count = detection.lidar_points_count;
    
    // Update chrono timestamps for compatibility
    track.last_seen = std::chrono::high_resolution_clock::now();
}

void MultiObjectTracker3D::createNewTracks(
    const std::vector<drive_yolo::DetectionWithDistance>& unassigned_detections,
    const ros::Time& timestamp) {
    
    for (const auto& detection : unassigned_detections) {
        if (detection.lidar_points_count >= min_lidar_points_ && detection.distance > 0) {
            auto new_track = std::make_shared<TrackedObject3D>(next_id_++, detection, expected_fusion_fps_);
            tracked_objects_[new_track->id] = new_track;
        }
    }
}

void MultiObjectTracker3D::removeOldTracks(const ros::Time& current_time) {
    auto it = tracked_objects_.begin();
    while (it != tracked_objects_.end()) {
        bool should_remove = false;
        
        // Remove if too many consecutive misses
        if (it->second->consecutive_misses > max_missed_frames_) {
            should_remove = true;
        }
        
        // Remove if object moved too far away
        float distance_from_origin = std::sqrt(
            it->second->position_3d.x * it->second->position_3d.x +
            it->second->position_3d.y * it->second->position_3d.y +
            it->second->position_3d.z * it->second->position_3d.z);
            
        if (distance_from_origin > max_tracking_distance_) {
            should_remove = true;
        }
        
        if (should_remove) {
            it = tracked_objects_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<TrackedObject3D> MultiObjectTracker3D::getAllTrackedObjects() const {
    std::vector<TrackedObject3D> result;
    result.reserve(tracked_objects_.size());
    
    for (const auto& [id, track] : tracked_objects_) {
        if (track->total_detections >= min_detections_for_tracking_) {
            result.push_back(*track);
        }
    }
    
    return result;
}

std::vector<TrackedObject3D> MultiObjectTracker3D::getHighQualityTracks() const {
    std::vector<TrackedObject3D> result;
    
    for (const auto& [id, track] : tracked_objects_) {
        if (track->isHighQualityTrack() && track->total_detections >= min_detections_for_tracking_) {
            result.push_back(*track);
        }
    }
    
    return result;
}

std::vector<TrackedObject3D> MultiObjectTracker3D::getMovingObjects(float min_speed_ms) const {
    std::vector<TrackedObject3D> result;
    
    for (const auto& [id, track] : tracked_objects_) {
        if (track->speed_3d >= min_speed_ms && track->total_detections >= min_detections_for_tracking_) {
            result.push_back(*track);
        }
    }
    
    return result;
}

std::vector<std::pair<int, int>> MultiObjectTracker3D::associateDetections3D(
    const std::vector<drive_yolo::DetectionWithDistance>& detections) {
    
    std::vector<std::pair<int, int>> associations;
    
    // Simple greedy association based on 3D distance
    std::vector<bool> track_used(tracked_objects_.size(), false);
    std::vector<bool> detection_used(detections.size(), false);
    
    // Create distance matrix and track IDs
    std::vector<std::vector<float>> distance_matrix;
    std::vector<int> track_ids;
    
    for (const auto& [id, track] : tracked_objects_) {
        track_ids.push_back(id);
        std::vector<float> row;
        for (const auto& detection : detections) {
            float dist = calculateDistance3D(*track, detection);
            row.push_back(dist);
        }
        distance_matrix.push_back(row);
    }
    
    // Greedy assignment
    while (true) {
        float min_distance = std::numeric_limits<float>::max();
        int best_track_idx = -1;
        int best_detection_idx = -1;
        
        for (size_t t = 0; t < distance_matrix.size(); ++t) {
            if (track_used[t]) continue;
            
            for (size_t d = 0; d < distance_matrix[t].size(); ++d) {
                if (detection_used[d]) continue;
                
                if (distance_matrix[t][d] < min_distance && 
                    distance_matrix[t][d] < distance_threshold_3d_) {
                    min_distance = distance_matrix[t][d];
                    best_track_idx = t;
                    best_detection_idx = d;
                }
            }
        }
        
        if (best_track_idx == -1) break;
        
        associations.push_back({track_ids[best_track_idx], best_detection_idx});
        track_used[best_track_idx] = true;
        detection_used[best_detection_idx] = true;
    }
    
    return associations;
}

float MultiObjectTracker3D::calculateDistance3D(const TrackedObject3D& track, 
                                               const drive_yolo::DetectionWithDistance& detection) const {
    cv::Point3f det_pos = extractPosition3D(detection);
    
    float dx = track.position_3d.x - det_pos.x;
    float dy = track.position_3d.y - det_pos.y;
    float dz = track.position_3d.z - det_pos.z;
    
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

cv::Point3f MultiObjectTracker3D::extractPosition3D(
    const drive_yolo::DetectionWithDistance& detection) const {
    
    if (detection.distance > 0 && detection.lidar_points_count > 0) {
        return cv::Point3f(detection.object_center_3d.x, 
                          detection.object_center_3d.y, 
                          detection.object_center_3d.z);
    } else {
        float estimated_distance = detection.distance > 0 ? static_cast<float>(detection.distance) : 5.0f;
        return cv::Point3f(detection.detection.x * 0.01f, 
                          detection.detection.y * 0.01f, 
                          estimated_distance);
    }
}

bool MultiObjectTracker3D::isValidDetection(const drive_yolo::DetectionWithDistance& detection) const {
    if (detection.detection.confidence < 0.2f) return false;
    if (detection.distance < 0.1 || detection.distance > max_tracking_distance_) return false;
    if (detection.lidar_points_count < 3) return false;
    if (detection.distance_confidence < 0.1f) return false;
    
    return true;
}

// ADDED: Missing methods implementation
int MultiObjectTracker3D::getActiveTracks() const {
    int count = 0;
    for (const auto& [id, track] : tracked_objects_) {
        if (track->total_detections >= min_detections_for_tracking_ && 
            track->consecutive_misses <= max_missed_frames_ / 2) {
            count++;
        }
    }
    return count;
}

float MultiObjectTracker3D::getAverageTrackingDistance() const {
    if (tracked_objects_.empty()) return 0.0f;
    
    float total_distance = 0.0f;
    int count = 0;
    
    for (const auto& [id, track] : tracked_objects_) {
        if (track->total_detections >= min_detections_for_tracking_) {
            float dist = std::sqrt(track->position_3d.x * track->position_3d.x +
                                 track->position_3d.y * track->position_3d.y +
                                 track->position_3d.z * track->position_3d.z);
            total_distance += dist;
            count++;
        }
    }
    
    return count > 0 ? total_distance / count : 0.0f;
}