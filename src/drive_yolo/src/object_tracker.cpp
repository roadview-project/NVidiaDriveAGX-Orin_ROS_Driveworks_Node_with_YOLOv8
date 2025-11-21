#include "drive_yolo/object_tracker.hpp"
#include <algorithm>
#include <cmath>

TrackedObject::TrackedObject(int obj_id, const drive_yolo::Detection& detection) 
    : id(obj_id)
    , class_id(detection.class_id)
    , class_name(detection.class_name)
    , position(detection.x, detection.y)
    , velocity(0.0f, 0.0f)
    , speed(0.0f)
    , size(detection.width, detection.height)
    , confidence(detection.confidence)
    , consecutive_misses(0)
    , total_detections(1)
    , kalman_initialized(false)
{
    auto now = std::chrono::high_resolution_clock::now();
    last_seen = now;
    first_seen = now;
    
    position_history.reserve(10);
    time_history.reserve(10);
    
    addToHistory(position, now);
}

void TrackedObject::updateKalman(const cv::Point2f& measured_pos, double dt) {
    if (!kalman_initialized) {
        // Initialize Kalman filter
        kalman.init(4, 2, 0);  // 4 state variables (x, y, vx, vy), 2 measurements (x, y)
        
        // State transition matrix (constant velocity model)
        kalman.transitionMatrix = (cv::Mat_<float>(4, 4) << 
            1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1);
        
        // Measurement matrix
        kalman.measurementMatrix = (cv::Mat_<float>(2, 4) << 
            1, 0, 0, 0,
            0, 1, 0, 0);
        
        // Process noise covariance
        float process_noise = 0.1f;
        kalman.processNoiseCov = (cv::Mat_<float>(4, 4) << 
            process_noise, 0, 0, 0,
            0, process_noise, 0, 0,
            0, 0, process_noise, 0,
            0, 0, 0, process_noise);
        
        // Measurement noise covariance
        float measurement_noise = 10.0f;
        kalman.measurementNoiseCov = (cv::Mat_<float>(2, 2) << 
            measurement_noise, 0,
            0, measurement_noise);
        
        // Error covariance
        kalman.errorCovPost = cv::Mat::eye(4, 4, CV_32F) * 1000;
        
        // Initial state
        kalman.statePre = (cv::Mat_<float>(4, 1) << measured_pos.x, measured_pos.y, 0, 0);
        kalman.statePost = kalman.statePre.clone();
        
        kalman_initialized = true;
    } else {
        // Update transition matrix with current dt
        kalman.transitionMatrix.at<float>(0, 2) = dt;
        kalman.transitionMatrix.at<float>(1, 3) = dt;
        
        // Predict
        cv::Mat prediction = kalman.predict();
        
        // Correct with measurement
        cv::Mat measurement = (cv::Mat_<float>(2, 1) << measured_pos.x, measured_pos.y);
        kalman.correct(measurement);
        
        // Update velocity from Kalman state
        velocity.x = kalman.statePost.at<float>(2, 0);
        velocity.y = kalman.statePost.at<float>(3, 0);
        speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
        
        // Update position from Kalman state (smoothed)
        position.x = kalman.statePost.at<float>(0, 0);
        position.y = kalman.statePost.at<float>(1, 0);
    }
}

void TrackedObject::predictKalman(double dt) {
    if (kalman_initialized) {
        kalman.transitionMatrix.at<float>(0, 2) = dt;
        kalman.transitionMatrix.at<float>(1, 3) = dt;
        
        cv::Mat prediction = kalman.predict();
        position.x = prediction.at<float>(0, 0);
        position.y = prediction.at<float>(1, 0);
        velocity.x = prediction.at<float>(2, 0);
        velocity.y = prediction.at<float>(3, 0);
        speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    }
}

cv::Point2f TrackedObject::getSmoothedVelocity() const {
    if (position_history.size() < 2) return cv::Point2f(0, 0);
    
    // Use linear regression or simple difference for smoothed velocity
    size_t n = std::min(position_history.size(), static_cast<size_t>(5)); // Use last 5 points
    if (n < 2) return cv::Point2f(0, 0);
    
    cv::Point2f vel(0, 0);
    float total_time = 0;
    
    for (size_t i = position_history.size() - n; i < position_history.size() - 1; ++i) {
        auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
            time_history[i + 1] - time_history[i]).count() / 1e6;
        
        if (dt > 0) {
            vel.x += (position_history[i + 1].x - position_history[i].x) / dt;
            vel.y += (position_history[i + 1].y - position_history[i].y) / dt;
            total_time += dt;
        }
    }
    
    if (total_time > 0) {
        vel.x /= (n - 1);
        vel.y /= (n - 1);
    }
    
    return vel;
}

float TrackedObject::getSmoothedSpeed() const {
    cv::Point2f vel = getSmoothedVelocity();
    return std::sqrt(vel.x * vel.x + vel.y * vel.y);
}

void TrackedObject::addToHistory(const cv::Point2f& pos, const std::chrono::high_resolution_clock::time_point& time) {
    position_history.push_back(pos);
    time_history.push_back(time);
    
    // Keep only recent history
    size_t max_history = 10;
    if (position_history.size() > max_history) {
        position_history.erase(position_history.begin());
        time_history.erase(time_history.begin());
    }
}

MultiObjectTracker::MultiObjectTracker() 
    : next_id_(1)
    , max_missed_frames_(10)      // Keep tracks for 10 frames without detection
    , distance_threshold_(100.0f)  // 100 pixels maximum for association
    , min_detections_for_tracking_(3)  // Need 3 detections before publishing
    , history_size_(10)
{
}

void MultiObjectTracker::updateTracks(const drive_yolo::Detections::ConstPtr& detections) {
    auto current_time = std::chrono::high_resolution_clock::now();
    
    // Calculate time delta from first detection (approximate)
    static auto first_detection_time = current_time;
    static bool first_time = true;
    if (first_time) {
        first_detection_time = current_time;
        first_time = false;
    }
    
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - first_detection_time).count() / 1e6;
    
    // Reset for next frame
    first_detection_time = current_time;
    
    // Predict all existing tracks
    predictTracks(dt);
    
    // Associate detections with existing tracks
    std::vector<drive_yolo::Detection> detection_vector(detections->detections.begin(), 
                                                        detections->detections.end());
    
    auto associations = associateDetections(detection_vector);
    
    // Update associated tracks
    std::vector<bool> detection_used(detection_vector.size(), false);
    for (const auto& [track_id, detection_idx] : associations) {
        auto track_it = tracked_objects_.find(track_id);
        if (track_it != tracked_objects_.end()) {
            updateTrack(*track_it->second, detection_vector[detection_idx]);
            detection_used[detection_idx] = true;
        }
    }
    
    // Create new tracks for unassigned detections
    std::vector<drive_yolo::Detection> unassigned_detections;
    for (size_t i = 0; i < detection_vector.size(); ++i) {
        if (!detection_used[i]) {
            unassigned_detections.push_back(detection_vector[i]);
        }
    }
    createNewTracks(unassigned_detections);
    
    // Remove old tracks
    removeOldTracks();
}

std::vector<std::pair<int, int>> MultiObjectTracker::associateDetections(
    const std::vector<drive_yolo::Detection>& detections) {
    
    std::vector<std::pair<int, int>> associations;
    
    // Simple greedy association based on distance
    std::vector<bool> track_used(tracked_objects_.size(), false);
    std::vector<bool> detection_used(detections.size(), false);
    
    // Create distance matrix
    std::vector<std::vector<float>> distance_matrix;
    std::vector<int> track_ids;
    
    for (const auto& [id, track] : tracked_objects_) {
        track_ids.push_back(id);
        std::vector<float> row;
        for (const auto& detection : detections) {
            float dist = calculateDistance(*track, detection);
            row.push_back(dist);
        }
        distance_matrix.push_back(row);
    }
    
    // Greedy assignment (could be improved with Hungarian algorithm)
    while (true) {
        float min_distance = std::numeric_limits<float>::max();
        int best_track_idx = -1;
        int best_detection_idx = -1;
        
        for (size_t t = 0; t < distance_matrix.size(); ++t) {
            if (track_used[t]) continue;
            
            for (size_t d = 0; d < distance_matrix[t].size(); ++d) {
                if (detection_used[d]) continue;
                
                if (distance_matrix[t][d] < min_distance && 
                    distance_matrix[t][d] < distance_threshold_) {
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

float MultiObjectTracker::calculateDistance(const TrackedObject& track, 
                                           const drive_yolo::Detection& detection) const {
    float dx = track.position.x - detection.x;
    float dy = track.position.y - detection.y;
    return std::sqrt(dx * dx + dy * dy);
}

void MultiObjectTracker::updateTrack(TrackedObject& track, const drive_yolo::Detection& detection) {
    auto current_time = std::chrono::high_resolution_clock::now();
    
    // Calculate time delta
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - track.last_seen).count() / 1e6;
    
    // Update Kalman filter
    track.updateKalman(cv::Point2f(detection.x, detection.y), dt);
    
    // Update other properties
    track.size = cv::Size2f(detection.width, detection.height);
    track.confidence = detection.confidence;
    track.last_seen = current_time;
    track.consecutive_misses = 0;
    track.total_detections++;
    
    // Add to history
    track.addToHistory(cv::Point2f(detection.x, detection.y), current_time);
}

void MultiObjectTracker::predictTracks(double dt) {
    for (auto& [id, track] : tracked_objects_) {
        track->predictKalman(dt);
        track->consecutive_misses++;
    }
}

void MultiObjectTracker::removeOldTracks() {
    auto it = tracked_objects_.begin();
    while (it != tracked_objects_.end()) {
        if (it->second->consecutive_misses > max_missed_frames_) {
            it = tracked_objects_.erase(it);
        } else {
            ++it;
        }
    }
}

void MultiObjectTracker::createNewTracks(const std::vector<drive_yolo::Detection>& unassigned_detections) {
    for (const auto& detection : unassigned_detections) {
        auto new_track = std::make_shared<TrackedObject>(next_id_++, detection);
        tracked_objects_[new_track->id] = new_track;
    }
}

std::vector<TrackedObject> MultiObjectTracker::getTrackedObjects() const {
    std::vector<TrackedObject> result;
    
    for (const auto& [id, track] : tracked_objects_) {
        // Only return tracks that have enough detections
        if (track->total_detections >= min_detections_for_tracking_) {
            result.push_back(*track);
        }
    }
    
    return result;
}