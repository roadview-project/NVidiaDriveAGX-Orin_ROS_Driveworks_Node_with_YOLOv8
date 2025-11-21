#!/usr/bin/env python3

import rospy
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Set
import time
import warnings
from collections import deque
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import DBSCAN

# Suppress motpy and other library warnings
warnings.filterwarnings('ignore')

# ROS messages - EXACT format matching your C++ messages
from drive_yolo.msg import (
    DetectionsWithDistance, DetectionWithDistance, Detection,
    TrackedObjects3D, TrackedObject3D
)
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header

# Tracking libraries
try:
    from motpy import MultiObjectTracker, Detection as MotpyDetection
    from motpy.utils import iou_distance, euclidean_distance
    HAS_MOTPY = True
except ImportError:
    HAS_MOTPY = False

try:
    from filterpy.kalman import KalmanFilter
    from filterpy.common import Q_discrete_white_noise
    HAS_FILTERPY = True
except ImportError:
    HAS_FILTERPY = False

@dataclass
class TrackedObject3DPython:
    """
    Enhanced Python TrackedObject3D with 1-second prediction capability
    Like a smart cache that keeps estimating object positions when GPS signal is lost
    """
    # Basic identification
    id: int
    class_id: int
    class_name: str
    last_prediction_update_time: float = 0.0
    # 3D position and motion (exactly matching your message)
    position_3d: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity_3d: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration_3d: np.ndarray = field(default_factory=lambda: np.zeros(3))
    speed_3d: float = 0.0
    
    # 2D bounding box info (for visualization)
    position_2d: np.ndarray = field(default_factory=lambda: np.zeros(2))
    size_2d: np.ndarray = field(default_factory=lambda: np.zeros(2))
    
    # Quality metrics (matching your message exactly)
    confidence: float = 0.0
    distance_confidence: float = 0.0
    lidar_points_count: int = 0
    track_stability: float = 0.0
    
    # Tracking info
    track_duration: float = 0.0
    total_detections: int = 0
    is_moving: bool = False
    predicted_position_1s: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Internal tracking state
    consecutive_misses: int = 0
    first_seen_time: float = field(default_factory=time.time)
    last_update_time: float = field(default_factory=time.time)
    
    # NEW: Prediction state management - like a "smart buffer" for lost objects
    is_predicting: bool = False                    # Currently in prediction mode
    prediction_start_time: float = 0.0            # When prediction started
    prediction_duration: float = 0.0              # How long we've been predicting
    MAX_PREDICTION_TIME: float = 1.0              # 1 second maximum prediction
    
    # NEW: Snapshot of state when prediction started (like saving checkpoint)
    prediction_start_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    prediction_start_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    prediction_start_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    last_real_detection_time: float = 0.0
    
    # Motion history for smoothing
    position_history: deque = field(default_factory=lambda: deque(maxlen=20))
    velocity_history: deque = field(default_factory=lambda: deque(maxlen=10))
    time_history: deque = field(default_factory=lambda: deque(maxlen=20))
    
    # Prediction feedback history - stores our predicted positions
    prediction_history: deque = field(default_factory=lambda: deque(maxlen=10))
    
    # Kalman filter state (for advanced tracking)
    kalman_filter: Optional[object] = None
    kalman_initialized: bool = False


    # NEW: Low-pass filter state for velocity smoothing
    filtered_velocity_3d: np.ndarray = field(default_factory=lambda: np.zeros(3))
    raw_velocity_3d: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Store raw for comparison
    filtered_acceleration_3d: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Filter parameters (loaded from ROS params)
    velocity_filter_alpha: float = 0.1      # Lower = more smoothing (0.1-0.5 recommended)
    acceleration_filter_alpha: float = 0.1  # Acceleration smoothing
    max_reasonable_velocity: float = 50.0   # Max velocity in m/s (outlier detection)
    max_reasonable_acceleration: float = 20.0  # Max acceleration in m/s²
    
    # Filter initialization flags
    velocity_filter_initialized: bool = False
    acceleration_filter_initialized: bool = False

    def _apply_velocity_low_pass_filter(self, raw_velocity: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to velocity to smooth out noise"""
        # Outlier detection - reject obviously wrong velocities
        velocity_magnitude = np.linalg.norm(raw_velocity)
        if velocity_magnitude > self.max_reasonable_velocity:
            rospy.logdebug(f"Track {self.id}: Rejecting outlier velocity {velocity_magnitude:.2f} m/s")
            # Use previous filtered velocity instead of the outlier
            raw_velocity = self.filtered_velocity_3d.copy()

        if not self.velocity_filter_initialized:
            # First measurement - initialize filter
            self.filtered_velocity_3d = raw_velocity.copy()
            self.velocity_filter_initialized = True
            return self.filtered_velocity_3d

        # Apply exponential moving average (low-pass filter)
        alpha = self.velocity_filter_alpha
        self.filtered_velocity_3d = (alpha * raw_velocity +
                                   (1.0 - alpha) * self.filtered_velocity_3d)

        # Optional: Additional smoothing for very noisy environments
        if hasattr(self, 'enable_extra_smoothing') and self.enable_extra_smoothing:
            extra_alpha = 0.3
            self.filtered_velocity_3d = (extra_alpha * self.filtered_velocity_3d +
                                       (1.0 - extra_alpha) * self.filtered_velocity_3d)

        return self.filtered_velocity_3d
    def load_filter_parameters_from_ros(self):
        """Load filter parameters from ROS parameter server"""
        self.velocity_filter_alpha = rospy.get_param('~velocity_filter_alpha', 0.1)
        self.acceleration_filter_alpha = rospy.get_param('~acceleration_filter_alpha', 0.1)
        self.max_reasonable_velocity = rospy.get_param('~max_reasonable_velocity', 50.0)
        self.max_reasonable_acceleration = rospy.get_param('~max_reasonable_acceleration', 20.0)
        self.enable_extra_smoothing = rospy.get_param('~enable_extra_velocity_smoothing', False)

    def _apply_acceleration_low_pass_filter(self, raw_acceleration: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to acceleration"""
        # Outlier detection for acceleration
        accel_magnitude = np.linalg.norm(raw_acceleration)
        if accel_magnitude > self.max_reasonable_acceleration:
            rospy.logdebug(f"Track {self.id}: Rejecting outlier acceleration {accel_magnitude:.2f} m/s²")
            raw_acceleration = self.filtered_acceleration_3d.copy()

        if not self.acceleration_filter_initialized:
            self.filtered_acceleration_3d = raw_acceleration.copy()
            self.acceleration_filter_initialized = True
            return self.filtered_acceleration_3d

        # Apply low-pass filter to acceleration
        alpha = self.acceleration_filter_alpha
        self.filtered_acceleration_3d = (alpha * raw_acceleration +
                                       (1.0 - alpha) * self.filtered_acceleration_3d)

        return self.filtered_acceleration_3d

    def update_with_detection(self, detection: DetectionWithDistance, timestamp: float):
        """Update track with new detection"""
        # If we were predicting, we're back to real data!
        if self.is_predicting:
            rospy.logdebug(f"Track {self.id} ({self.class_name}) recovered after {self.prediction_duration:.2f}s prediction")
            self._end_prediction_mode()

        # Extract 3D position from detection
        new_position = np.array([
            detection.object_center_3d.x,
            detection.object_center_3d.y,
            detection.object_center_3d.z
        ])

        # Check if this detection aligns reasonably with our prediction
        if self.prediction_history and len(self.position_history) > 0:
            last_predicted = self.prediction_history[-1] if self.prediction_history else self.position_3d
            prediction_error = np.linalg.norm(new_position - last_predicted)

            if prediction_error < 5.0:  # Within 5 meters is reasonable
                # Boost confidence since our prediction was good
                self.track_stability = min(1.0, self.track_stability + 0.1)
            else:
                rospy.logdebug(f"Track {self.id}: Large prediction error: {prediction_error:.2f}m")

        self.position_3d = new_position
        
        # Extract 2D info
        self.position_2d = np.array([detection.detection.x, detection.detection.y])
        self.size_2d = np.array([detection.detection.width, detection.detection.height])
        
        # Update quality metrics
        self.confidence = detection.detection.confidence
        self.distance_confidence = detection.distance_confidence
        self.lidar_points_count = detection.lidar_points_count
        self.class_id = detection.detection.class_id
        self.class_name = detection.detection.class_name
        
        # Add to history
        self.position_history.append(self.position_3d.copy())
        self.time_history.append(timestamp)
        
        # Update motion calculations
        self.velocity_3d = self._calculate_velocity_from_history()
        self.acceleration_3d = self._calculate_acceleration_from_history()
        self.speed_3d = float(np.linalg.norm(self.velocity_3d))
        
        # Update tracking state
        self.last_update_time = timestamp
        self.last_real_detection_time = timestamp  # NEW: Track last real detection
        self.consecutive_misses = 0
        self.total_detections += 1
        self.track_duration = timestamp - self.first_seen_time
        self.is_moving = self.speed_3d > 0.1
        
        # Calculate track stability
        self.track_stability = self._calculate_track_stability()
        
        # Predict future position
        self.predicted_position_1s = self.predict_position_after(1.0)
        
        # Store velocity in history
        self.velocity_history.append(self.velocity_3d.copy())
    def increment_miss_counter(self, current_time: float):
        """
        Called when track wasn't matched - now with prediction capability
        """
        self.consecutive_misses += 1
        
        # FIX: Only start prediction on first miss AND if not already predicting
        if self.consecutive_misses == 1 and not self.is_predicting:
            self._start_prediction_mode(current_time)
        
        # FIX: Always update prediction if in prediction mode
        # (not just when is_predicting is true)
        if self.is_predicting:
            self._update_prediction(current_time)
    

    def _start_prediction_mode(self, current_time: float):
        """Start 1-second prediction mode"""
        # Check if already predicting
        if self.is_predicting:
            return

        # Even with minimal data, we can predict
        if np.any(self.position_3d):  # As long as we have a position
            self.is_predicting = True
            self.prediction_start_time = current_time
            self.prediction_duration = 0.0

            # Save current state as prediction starting point
            self.prediction_start_position = self.position_3d.copy()
            self.prediction_start_velocity = self.velocity_3d.copy() if np.any(self.velocity_3d) else np.zeros(3)
            self.prediction_start_acceleration = self.acceleration_3d.copy() if np.any(self.acceleration_3d) else np.zeros(3)

            # Clear prediction history for this new sequence
            self.prediction_history.clear()

            rospy.logdebug(f"Track {self.id} ({self.class_name}) starting prediction at speed={self.speed_3d:.2f}m/s")


    def _update_prediction(self, current_time: float):
        """Update predicted position"""
        if not self.is_predicting:
            return

        # Calculate how long we've been predicting
        self.prediction_duration = current_time - self.prediction_start_time

        # Stop predicting after 1 second
        if self.prediction_duration >= self.MAX_PREDICTION_TIME:
            rospy.logdebug(f"Track {self.id} prediction timeout after {self.prediction_duration:.2f}s")
            self._end_prediction_mode()
            return

        # Predict new position using physics equations
        dt = self.prediction_duration

        # Kinematic equation: position = initial_pos + velocity*t + 0.5*acceleration*t^2
        predicted_position = (
            self.prediction_start_position +
            self.prediction_start_velocity * dt +
            0.5 * self.prediction_start_acceleration * dt * dt
        )

        # Update our current position
        self.position_3d = predicted_position

        # Update the timestamp so continuous publisher knows this is fresh
        self.last_prediction_update_time = current_time

        # Store this prediction for later validation
        self.prediction_history.append(predicted_position.copy())

        # Update 2D position proportionally (if needed)
        if np.any(self.position_2d):
            distance_ratio = 1.0
            if len(self.position_history) > 0:
                old_distance = np.linalg.norm(self.prediction_start_position)
                new_distance = np.linalg.norm(predicted_position)
                if old_distance > 0:
                    distance_ratio = new_distance / old_distance

            # Scale 2D bounding box based on predicted distance
            self.size_2d = self.size_2d * distance_ratio

        # Gradually reduce confidence during prediction
        confidence_decay = max(0.1, 1.0 - (dt / self.MAX_PREDICTION_TIME) * 0.5)
        self.confidence *= confidence_decay
        self.distance_confidence *= confidence_decay

        # Update predicted future position (1s ahead from current predicted position)
        self.predicted_position_1s = self.predict_position_after(1.0)

    def _end_prediction_mode(self):
        """End prediction mode - back to real data"""
        if not self.is_predicting:
            return

        rospy.logdebug(f"Track {self.id} ({self.class_name}) ending prediction after {self.prediction_duration:.2f}s")

        self.is_predicting = False
        self.prediction_duration = 0.0

        # Note: We keep prediction_history for validation against future detections

    def should_remove_track(self, current_time: float) -> bool:
        """Enhanced removal logic - tracks can live longer during prediction"""
        # If we're predicting and still within 1 second, keep the track
        if self.is_predicting and self.prediction_duration < self.MAX_PREDICTION_TIME:
            return False

        # If prediction time exceeded, check if we should remove
        if self.is_predicting and self.prediction_duration >= self.MAX_PREDICTION_TIME:
            self._end_prediction_mode()
            # After prediction ends, immediately remove if it hasn't been seen
            return True

        # Check if track has moved behind the vehicle (negative X in camera frame)
        # This catches predictions that went backward
        if self.position_3d[0] < -2.0:  # More than 2m behind camera
            return True

        # Normal removal logic (but more lenient for tracks that were predicting)
        max_misses = 8  # Standard threshold
        if self.prediction_history:  # Had predictions, be more lenient
            max_misses = 10  # Slightly more lenient (was 12)

        return self.consecutive_misses > max_misses

    def get_debug_info(self) -> str:
        """
        Get debug information about track state
        """
        status = "TRACKING"
        if self.is_predicting:
            status = f"PREDICTING({self.prediction_duration:.2f}s)"
        elif self.consecutive_misses > 0:
            status = f"MISSING({self.consecutive_misses})"
            
        return (f"Track {self.id} [{status}]: "
                f"pos=[{self.position_3d[0]:.1f},{self.position_3d[1]:.1f},{self.position_3d[2]:.1f}] "
                f"speed={self.speed_3d:.1f}m/s conf={self.confidence:.2f}")


    
    def _calculate_velocity_from_history(self) -> np.ndarray:
        """
        Enhanced velocity calculation with filtering
        """
        if len(self.position_history) < 2:
            return np.zeros(3)
        
        # Calculate raw velocity (existing logic)
        n_points = min(5, len(self.position_history))
        if n_points < 2:
            return np.zeros(3)
        
        velocities = []
        weights = []
        
        # Calculate velocities between consecutive points
        for i in range(-n_points+1, 0):
            pos_curr = self.position_history[i]
            pos_prev = self.position_history[i-1]
            time_curr = self.time_history[i]
            time_prev = self.time_history[i-1]
            
            dt = time_curr - time_prev
            if dt > 0.01:  # Valid time interval
                velocity = (pos_curr - pos_prev) / dt
                velocities.append(velocity)
                # Weight recent samples more heavily
                weight = 1.0 + abs(i) * 0.3  # More recent = higher weight
                weights.append(weight)
        
        if not velocities:
            return np.zeros(3)
        
        # Weighted average of velocities (this is the "raw" velocity)
        total_weight = sum(weights)
        raw_weighted_velocity = sum(v * w for v, w in zip(velocities, weights))
        self.raw_velocity_3d = raw_weighted_velocity / total_weight
        
        # Apply low-pass filter to smooth the velocity
        filtered_velocity = self._apply_velocity_low_pass_filter(self.raw_velocity_3d)

        return filtered_velocity

    def _calculate_acceleration_from_history(self) -> np.ndarray:
        """
        Enhanced acceleration calculation with filtering
        """
        if len(self.velocity_history) < 2:
            return np.zeros(3)
        
        # Calculate raw acceleration from velocity history
        if len(self.time_history) >= 2:
            v_curr = self.velocity_history[-1]
            v_prev = self.velocity_history[-2]
            t_curr = self.time_history[-1] 
            t_prev = self.time_history[-2]
            
            dt = t_curr - t_prev
            if dt > 0.01:
                raw_acceleration = (v_curr - v_prev) / dt
                
                # NEW: Apply low-pass filter to acceleration
                filtered_acceleration = self._apply_acceleration_low_pass_filter(raw_acceleration)
                return filtered_acceleration
        
        return np.zeros(3)
    
    def _calculate_track_stability(self) -> float:
        """Calculate track stability [0,1] based on multiple factors"""
        stability = 0.0
        
        # Factor 1: Detection consistency (30% weight)
        detection_consistency = min(1.0, self.total_detections / 10.0)
        stability += 0.3 * detection_consistency
        
        # Factor 2: Recent activity (20% weight) 
        miss_factor = max(0.0, 1.0 - self.consecutive_misses / 5.0)
        stability += 0.2 * miss_factor
        
        # Factor 3: LiDAR data quality (25% weight)
        lidar_quality = min(1.0, self.lidar_points_count / 20.0)
        stability += 0.15 * lidar_quality
        stability += 0.1 * self.distance_confidence
        
        # Factor 4: Detection confidence (15% weight)
        stability += 0.15 * self.confidence
        
        # Factor 5: Motion consistency (10% weight)
        if len(self.velocity_history) >= 3:
            recent_speeds = [np.linalg.norm(v) for v in list(self.velocity_history)[-3:]]
            speed_variance = np.var(recent_speeds) if len(recent_speeds) > 1 else 0
            motion_consistency = max(0.0, 1.0 - speed_variance / 2.0)
            stability += 0.1 * motion_consistency
        
        # NEW: Bonus for successful predictions
        if self.prediction_history:
            prediction_bonus = min(0.1, len(self.prediction_history) / 10.0)
            stability += prediction_bonus
        
        return min(1.0, stability)
    
    def is_high_quality_track(self) -> bool:
        """Determine if this is a high-quality track"""
        return (self.total_detections >= 3 and 
                self.consecutive_misses <= 2 and
                self.lidar_points_count >= 5 and
                self.distance_confidence > 0.5 and
                self.confidence > 0.3 and
                self.track_stability > 0.6)
    
    def predict_position_after(self, time_seconds: float) -> np.ndarray:
        """Predict position after given time using current motion"""
        predicted = self.position_3d.copy()
        
        # Linear prediction with velocity
        predicted += self.velocity_3d * time_seconds
        
        # Add quadratic term with acceleration
        predicted += 0.5 * self.acceleration_3d * time_seconds**2
        
        return predicted


class MultiObjectTracker3DPython:
    """
    Enhanced 3D tracker with 1-second prediction capability
    """
    
    def __init__(self):
        self.tracked_objects: Dict[int, TrackedObject3DPython] = {}
        self.next_id = 1
        
        # Load ROS parameters
        self.distance_threshold_3d = rospy.get_param('~distance_threshold_3d', 2.0)
        self.max_missed_frames = rospy.get_param('~max_missed_frames', 8)
        self.min_detections_for_tracking = rospy.get_param('~min_detections_for_tracking', 3)
        self.min_lidar_points = rospy.get_param('~min_lidar_points', 5)
        self.max_tracking_distance = rospy.get_param('~max_tracking_distance', 200.0)
        self.min_speed_threshold = rospy.get_param('~min_speed_threshold', 0.1)
        self.velocity_filter_alpha = rospy.get_param('~velocity_filter_alpha', 0.1)
        self.acceleration_filter_alpha = rospy.get_param('~acceleration_filter_alpha', 0.1)
        self.max_reasonable_velocity = rospy.get_param('~max_reasonable_velocity', 50.0)
        self.max_reasonable_acceleration = rospy.get_param('~max_reasonable_acceleration', 20.0)
        
        # Initialize advanced association
        self.associator = AdvancedAssociationTracker()

        # Rain/noise filtering parameters
        self.min_detection_distance = rospy.get_param('~min_detection_distance', 1.5)  # Reject detections < 1.5m (likely rain)
        self.max_bbox_area_for_noise = rospy.get_param('~max_bbox_area_for_noise', 5000)  # Small bbox = noise/rain
        self.min_bbox_area = rospy.get_param('~min_bbox_area', 100)  # Too small = noise
        self.enable_rain_filter = rospy.get_param('~enable_rain_filter', True)

        # Statistics for prediction performance
        self.prediction_stats = {
            'total_predictions': 0,
            'successful_recoveries': 0,
            'prediction_errors': [],
            'rain_filtered': 0
        }

        rospy.loginfo("3D Tracker configured with prediction capability")
        rospy.loginfo(f"Distance threshold: {self.distance_threshold_3d}m")
        rospy.loginfo(f"Rain filter: {'ENABLED' if self.enable_rain_filter else 'DISABLED'} (min dist: {self.min_detection_distance}m)")
        rospy.loginfo(f"Min detections for tracking: {self.min_detections_for_tracking}")
    
    def update_tracks(self, detections_msg: DetectionsWithDistance) -> None:
        """
        Main tracking update method with prediction support
        """
        current_time = time.time()
        
        # Filter valid detections
        valid_detections = self._filter_valid_detections(detections_msg.detections)
        
        # Update with custom tracker (enhanced with prediction)
        self._update_with_enhanced_tracker(valid_detections, current_time)
        
        # Remove old tracks (but respect prediction timing)
        self._remove_old_tracks(current_time)
    
    def _update_with_enhanced_tracker(self, detections: List[DetectionWithDistance], current_time: float):
        """
        Update using custom tracker with prediction capability
        """
        
        # Associate detections with existing tracks (including predicted ones)
        associations, unmatched_detections = self.associator.associate_detections(
            self.tracked_objects, detections, self.distance_threshold_3d)
        
        # Update matched tracks
        for track_id, det_idx in associations:
            if track_id in self.tracked_objects:
                track = self.tracked_objects[track_id]
                
                # If this track was predicting, we found it again!
                if track.is_predicting:
                    self.prediction_stats['successful_recoveries'] += 1
                    rospy.logdebug(f"Track {track_id} recovered after {track.prediction_duration:.2f}s prediction")
                
                track.update_with_detection(detections[det_idx], current_time)
        
        # Handle unmatched tracks (start or continue prediction)
        matched_track_ids = {track_id for track_id, _ in associations}
        for track_id, track in self.tracked_objects.items():
            if track_id not in matched_track_ids:
                track.increment_miss_counter(current_time)
                
                # NEW: Track prediction statistics
                if track.is_predicting:
                    self.prediction_stats['total_predictions'] += 1
        
        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            detection = detections[det_idx]
            new_track = TrackedObject3DPython(
                id=self.next_id,
                class_id=detection.detection.class_id,
                class_name=detection.detection.class_name
            )
            new_track.velocity_filter_alpha = self.velocity_filter_alpha
            new_track.acceleration_filter_alpha = self.acceleration_filter_alpha
            new_track.max_reasonable_velocity = self.max_reasonable_velocity
            new_track.max_reasonable_acceleration = self.max_reasonable_acceleration
            new_track.load_filter_parameters_from_ros()
            new_track.update_with_detection(detection, current_time)
            self.tracked_objects[self.next_id] = new_track
            self.next_id += 1
    
    def _remove_old_tracks(self, current_time: float):
        """
        Remove old tracks (but respect prediction timing)
        """
        to_remove = []
        
        for track_id, track in self.tracked_objects.items():
            if track.should_remove_track(current_time):
                # Log removal reason
                if track.is_predicting:
                    rospy.logdebug(f"🔮 Removing track {track_id} after prediction timeout")
                else:
                    rospy.logdebug(f"Removing track {track_id} after {track.consecutive_misses} misses")
                to_remove.append(track_id)
        
        for track_id in to_remove:
            del self.tracked_objects[track_id]
    
    def _is_likely_rain_or_noise(self, detection: DetectionWithDistance) -> bool:
        """
        Detect if a detection is likely rain, spray, or noise
        Rain characteristics:
        - Very close to camera (< 1.5-2m)
        - Small bounding box
        - Low LiDAR points
        - Often appears in clusters
        """
        if not self.enable_rain_filter:
            return False

        # Calculate bounding box area
        bbox_area = detection.detection.width * detection.detection.height

        # Check for rain characteristics
        is_too_close = detection.distance < self.min_detection_distance
        is_very_small = bbox_area < self.min_bbox_area
        is_low_confidence = detection.distance_confidence < 0.3
        is_few_lidar_points = detection.lidar_points_count < 3

        # Rain is typically: very close + (small OR low confidence OR few points)
        if is_too_close and (is_very_small or is_low_confidence or is_few_lidar_points):
            return True

        # Also filter very small detections at any distance (noise)
        if is_very_small and detection.detection.confidence < 0.4:
            return True

        return False

    def _filter_valid_detections(self, detections: List[DetectionWithDistance]) -> List[DetectionWithDistance]:
        """Filter detections based on quality criteria and rain filtering"""
        valid_detections = []
        rain_count = 0

        for det in detections:
            # Skip rain/noise detections
            if self._is_likely_rain_or_noise(det):
                rain_count += 1
                continue

            # Standard quality checks
            if (det.distance > self.min_detection_distance and
                det.distance <= self.max_tracking_distance and
                det.lidar_points_count >= self.min_lidar_points and
                det.distance_confidence > 0.1 and
                det.detection.confidence > 0.2):
                valid_detections.append(det)

        # Track rain filtering statistics
        if rain_count > 0:
            self.prediction_stats['rain_filtered'] += rain_count

        return valid_detections
    
    def get_all_tracked_objects(self) -> List[TrackedObject3DPython]:
        """Get all tracked objects that meet minimum requirements"""
        return [
            track for track in self.tracked_objects.values()
            if track.total_detections >= self.min_detections_for_tracking
        ]
    
    def get_high_quality_tracks(self) -> List[TrackedObject3DPython]:
        """Get only high-quality tracks"""
        return [
            track for track in self.tracked_objects.values()
            if (track.total_detections >= self.min_detections_for_tracking and 
                track.is_high_quality_track())
        ]
    
    def get_moving_objects(self, min_speed: float = None) -> List[TrackedObject3DPython]:
        """Get objects moving faster than threshold"""
        if min_speed is None:
            min_speed = self.min_speed_threshold
            
        return [
            track for track in self.tracked_objects.values()
            if (track.total_detections >= self.min_detections_for_tracking and 
                track.speed_3d >= min_speed)
        ]
    
    def get_stats(self) -> Dict[str, int]:
        """Get tracker statistics including prediction stats"""
        all_tracks = list(self.tracked_objects.values())
        predicting_tracks = [t for t in all_tracks if t.is_predicting]
        
        stats = {
            'total_tracks': len(all_tracks),
            'active_tracks': len([t for t in all_tracks if t.consecutive_misses <= 2]),
            'high_quality_tracks': len([t for t in all_tracks if t.is_high_quality_track()]),
            'moving_objects': len([t for t in all_tracks if t.speed_3d >= self.min_speed_threshold]),
            'predicting_tracks': len(predicting_tracks),  # NEW
            'total_predictions': self.prediction_stats['total_predictions'],  # NEW
            'successful_recoveries': self.prediction_stats['successful_recoveries']  # NEW
        }
        
        return stats


# Keep your existing AdvancedAssociationTracker class unchanged...
class AdvancedAssociationTracker:
    """
    Advanced data association using Hungarian algorithm + multiple distance metrics
    """
    
    def __init__(self):
        self.distance_weights = {
            'euclidean_3d': 0.4,    
            'velocity_diff': 0.25,   
            'size_similarity': 0.15, 
            'class_match': 0.2       
        }
    
    def associate_detections(self, 
                           tracks: Dict[int, TrackedObject3DPython], 
                           detections: List[DetectionWithDistance],
                           max_distance: float = 3.0) -> Tuple[List[Tuple[int, int]], Set[int]]:
        """Enhanced association that considers predicted positions"""
        
        if not tracks or not detections:
            return [], set(range(len(detections)))
        
        track_ids = list(tracks.keys())
        n_tracks = len(track_ids)
        n_detections = len(detections)
        
        # Create cost matrix [tracks x detections]
        cost_matrix = np.full((n_tracks, n_detections), np.inf)
        
        for i, track_id in enumerate(track_ids):
            track = tracks[track_id]
            
            for j, detection in enumerate(detections):
                # Calculate combined distance metric
                total_cost = self._calculate_association_cost(track, detection)
                
                # NEW: Be more lenient with predicted tracks
                adjusted_max_distance = max_distance
                if track.is_predicting:
                    # Allow larger association distance for predicted tracks
                    adjusted_max_distance = max_distance * 1.5
                    rospy.logdebug(f"Track {track_id} predicting: using relaxed distance {adjusted_max_distance}")
                
                if total_cost < adjusted_max_distance:
                    cost_matrix[i, j] = total_cost
        
        # Solve assignment problem using Hungarian algorithm
        valid_assignments = cost_matrix < max_distance * 2  # Even more lenient check
        if not np.any(valid_assignments):
            return [], set(range(len(detections)))
        
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
        
        associations = []
        matched_detections = set()
        
        for track_idx, det_idx in zip(row_indices, col_indices):
            # Use the adjusted max distance for final check
            track_id = track_ids[track_idx]
            track = tracks[track_id]
            adjusted_max = max_distance * 1.5 if track.is_predicting else max_distance
            
            if cost_matrix[track_idx, det_idx] < adjusted_max:
                associations.append((track_id, det_idx))
                matched_detections.add(det_idx)
        
        unmatched_detections = set(range(len(detections))) - matched_detections
        
        return associations, unmatched_detections
    
    def _calculate_association_cost(self,
                                  track: TrackedObject3DPython,
                                  detection: DetectionWithDistance) -> float:
        """Calculate comprehensive association cost between track and detection"""

        # Extract detection 3D position
        det_pos = np.array([
            detection.object_center_3d.x,
            detection.object_center_3d.y,
            detection.object_center_3d.z
        ])

        # Use current position (which might be predicted)
        track_pos = track.position_3d

        # 1. Euclidean 3D distance
        euclidean_dist = np.linalg.norm(track_pos - det_pos)

        # SANITY CHECK: Reject associations with impossible implied velocities
        # If the object moved more than 20m in 0.1s, that's 200 m/s = 720 km/h - impossible!
        dt = 0.1  # Assume 10Hz update rate
        if euclidean_dist > 20.0:  # More than 20m movement in one frame
            implied_velocity = euclidean_dist / dt
            # If track has established velocity, check if this would be a huge jump
            if np.linalg.norm(track.velocity_3d) > 1.0:  # Track is moving
                velocity_jump = abs(implied_velocity - np.linalg.norm(track.velocity_3d))
                if velocity_jump > 50.0:  # Velocity jump > 180 km/h in one frame = reject
                    return np.inf  # Reject this association

        euclidean_cost = euclidean_dist * self.distance_weights['euclidean_3d']

        # 2. Velocity prediction consistency
        predicted_pos = track_pos + track.velocity_3d * dt
        velocity_dist = np.linalg.norm(predicted_pos - det_pos)
        velocity_cost = velocity_dist * self.distance_weights['velocity_diff']

        # 3. Size similarity (2D bounding box)
        det_size = np.array([detection.detection.width, detection.detection.height])
        if np.any(track.size_2d) and np.any(det_size):
            size_ratio = np.abs(1 - (det_size / (track.size_2d + 1e-6)))
            size_cost = np.mean(size_ratio) * self.distance_weights['size_similarity']
        else:
            size_cost = 0.5 * self.distance_weights['size_similarity']

        # 4. Class consistency bonus/penalty
        if track.class_id == detection.detection.class_id:
            class_cost = 0  # Perfect match
        else:
            class_cost = 2.0 * self.distance_weights['class_match']

        # 5. Prediction penalty - if track is predicting, add small cost
        prediction_penalty = 0.0
        if track.is_predicting:
            # Small penalty that increases with prediction time
            prediction_penalty = 0.5 * (track.prediction_duration / track.MAX_PREDICTION_TIME)

        total_cost = euclidean_cost + velocity_cost + size_cost + class_cost + prediction_penalty

        return total_cost


class Tracking3DNode:
    """
    Enhanced ROS node for 3D object tracking with continuous prediction publishing
    Like a heartbeat system that keeps downstream components fed
    """
    
    def __init__(self):
        rospy.init_node('tracking_3d_python_node', anonymous=True)
        
        # Initialize tracker
        self.tracker = MultiObjectTracker3DPython()
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_stats_report = time.time()
        self.total_processing_time = 0.0
        
        # Get additional parameters
        self.quiet_mode = rospy.get_param('~quiet_mode', False)
        self.publish_all_tracks = rospy.get_param('~publish_all_tracks', True)
        self.publish_high_quality_only = rospy.get_param('~publish_high_quality_only', False)
        
        # NEW: Continuous publishing parameters
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # 10 Hz default
        self.enable_continuous_publishing = rospy.get_param('~enable_continuous_publishing', True)
        
        # ROS communication setup
        self.detections_sub = rospy.Subscriber(
            '/detections_with_distance', 
            DetectionsWithDistance, 
            self.detections_callback, 
            queue_size=10
        )
        
        # Publishers - exactly matching your C++ node
        self.tracked_objects_pub = rospy.Publisher(
            '/tracked_objects_3d', TrackedObjects3D, queue_size=10)
        
        self.high_quality_tracks_pub = rospy.Publisher(
            '/high_quality_tracks_3d', TrackedObjects3D, queue_size=10)
        
        self.moving_objects_pub = rospy.Publisher(
            '/moving_objects_3d', TrackedObjects3D, queue_size=10)
        
        # NEW: Continuous publishing timer - the heartbeat system
        if self.enable_continuous_publishing:
            self.publish_timer = rospy.Timer(
                rospy.Duration(1.0 / self.publish_rate), 
                self.continuous_publish_callback
            )
            rospy.loginfo(f"🔄 Continuous publishing enabled at {self.publish_rate} Hz")
        
        # NEW: Track last header for continuous publishing
        self.last_header = Header()
        self.last_header.frame_id = "camera"  # Default frame
        self.publishing_stats = {
            'continuous_publishes': 0,
            'detection_driven_publishes': 0
        }
        
        # Display startup info
        if not self.quiet_mode:
            rospy.loginfo("=== PYTHON 3D TRACKER INITIALIZED ===")
            rospy.loginfo(f"Distance threshold: {self.tracker.distance_threshold_3d}m | Max missed: {self.tracker.max_missed_frames}")
            rospy.loginfo(f"Min detections: {self.tracker.min_detections_for_tracking} | Min LiDAR points: {self.tracker.min_lidar_points}")
            rospy.loginfo(f"Prediction: 1.0s max | Continuous publishing: {self.publish_rate} Hz")
            rospy.loginfo(f"Rain filter: {'ON' if self.tracker.enable_rain_filter else 'OFF'} (min dist: {self.tracker.min_detection_distance}m)")
            rospy.loginfo("Topics: /detections_with_distance -> /tracked_objects_3d, /high_quality_tracks_3d, /moving_objects_3d")
            rospy.loginfo("====================================")
        else:
            rospy.loginfo("Python 3D Tracking Node initialized (QUIET MODE)")
    
    def continuous_publish_callback(self, timer_event):
        """
        Timer callback for continuous publishing - enhanced version
        """
        current_time = time.time()
        
        # Update predictions for all tracks
        self._update_predictions_only(current_time)
        
        # Get current tracked objects (including predicted positions)
        all_tracks = self.tracker.get_all_tracked_objects()
        
        # FIX: Always publish, even with empty tracks (send empty message)
        # This maintains the heartbeat even when no objects are tracked
        
        high_quality_tracks = self.tracker.get_high_quality_tracks()
        moving_objects = self.tracker.get_moving_objects()
        
        # Create a header with current timestamp
        current_header = Header()
        current_header.stamp = rospy.Time.now()
        current_header.frame_id = self.last_header.frame_id if self.last_header else "camera"
        
        # Publish all track types - maintains heartbeat
        if self.publish_all_tracks:
            self._publish_tracked_objects(all_tracks, current_header, 0.0, 
                                        self.tracked_objects_pub, is_continuous=True)
        
        if self.publish_high_quality_only:
            self._publish_tracked_objects(high_quality_tracks, current_header, 0.0, 
                                        self.tracked_objects_pub, is_continuous=True)
        
        # Always publish filtered results (even if empty)
        self._publish_tracked_objects(high_quality_tracks, current_header, 0.0, 
                                    self.high_quality_tracks_pub, is_continuous=True)
        self._publish_tracked_objects(moving_objects, current_header, 0.0, 
                                    self.moving_objects_pub, is_continuous=True)
        
        # Update stats
        self.publishing_stats['continuous_publishes'] += 1
        
        # Minimal logging for continuous publishing
        if not self.quiet_mode and all_tracks:
            predicting_tracks = [t for t in all_tracks if t.is_predicting]

            # Log only occasionally during active prediction
            if predicting_tracks and self.publishing_stats['continuous_publishes'] % 100 == 0:
                rospy.logdebug(f"Continuous publish: {len(all_tracks)} tracks ({len(predicting_tracks)} predicting)")


    
    def _update_predictions_only(self, current_time: float):
        """
        Update predictions for existing tracks without processing new detections
        This is called every 100ms by the continuous publisher
        """
        for track_id, track in list(self.tracker.tracked_objects.items()):
            # Check how long since last real update
            time_since_last_update = current_time - track.last_update_time
            
            # If more than 100ms since last update and not predicting, start prediction
            if time_since_last_update > 0.1 and not track.is_predicting:
                # Track hasn't been updated recently, should start predicting
                if track.consecutive_misses == 0:
                    # First time we notice it's missing
                    track.consecutive_misses = 1
                track._start_prediction_mode(current_time)
                
            # If already predicting, update the prediction
            if track.is_predicting:
                track._update_prediction(current_time)
        
        # Remove tracks that have exceeded their time
        self._remove_expired_tracks(current_time)

        
    def _remove_expired_tracks(self, current_time: float):
        """
        Remove tracks that have exceeded their prediction time
        More aggressive than the normal removal in update_tracks
        """
        to_remove = []
        
        for track_id, track in self.tracker.tracked_objects.items():
            if track.should_remove_track(current_time):
                to_remove.append(track_id)
        
        for track_id in to_remove:
            if not self.quiet_mode:
                track = self.tracker.tracked_objects[track_id]
                reason = "prediction timeout" if track.is_predicting else f"{track.consecutive_misses} misses"
                rospy.logdebug(f"🔄 Continuous mode: Removing track {track_id} ({reason})")
            del self.tracker.tracked_objects[track_id]
    
    def detections_callback(self, msg: DetectionsWithDistance):
        """
        Handle incoming detections - this is the "real data" path
        Now works alongside continuous publishing
        """
        callback_start = time.time()
        
        # Store the header for continuous publishing
        self.last_header = msg.header
        
        # Update tracker with new detections (normal processing)
        self.tracker.update_tracks(msg)
        
        # Get different types of tracked objects
        all_tracks = self.tracker.get_all_tracked_objects()
        high_quality_tracks = self.tracker.get_high_quality_tracks()
        moving_objects = self.tracker.get_moving_objects()
        
        callback_end = time.time()
        processing_time_ms = (callback_end - callback_start) * 1000.0
        
        self.frame_count += 1
        self.total_processing_time += processing_time_ms
        
        # Publish results based on configuration (detection-driven publish)
        if self.publish_all_tracks:
            self._publish_tracked_objects(all_tracks, msg.header, processing_time_ms, 
                                        self.tracked_objects_pub, is_continuous=False)
        
        if self.publish_high_quality_only:
            self._publish_tracked_objects(high_quality_tracks, msg.header, processing_time_ms, 
                                        self.tracked_objects_pub, is_continuous=False)
        
        # Always publish filtered results
        self._publish_tracked_objects(high_quality_tracks, msg.header, processing_time_ms, 
                                    self.high_quality_tracks_pub, is_continuous=False)
        self._publish_tracked_objects(moving_objects, msg.header, processing_time_ms, 
                                    self.moving_objects_pub, is_continuous=False)
        
        # Update detection-driven publishing stats
        self.publishing_stats['detection_driven_publishes'] += 1
        
        # Minimal logging (every 30 frames to reduce spam)
        if not self.quiet_mode and all_tracks and self.frame_count % 30 == 0:
            predicting_tracks = [t for t in all_tracks if t.is_predicting]
            rain_filtered = self.tracker.prediction_stats.get('rain_filtered', 0)

            log_msg = (f"Frame {self.frame_count}: {len(all_tracks)} tracks ({len(high_quality_tracks)} HQ, "
                      f"{len(moving_objects)} moving")

            if predicting_tracks:
                log_msg += f", {len(predicting_tracks)} predicting"

            if rain_filtered > 0:
                log_msg += f", {rain_filtered} rain filtered"

            rospy.loginfo(log_msg)
        
        # Less frequent statistics report (every 30 seconds to reduce spam)
        current_time = time.time()
        if current_time - self.last_stats_report >= 30.0:
            self._report_enhanced_statistics(current_time, all_tracks, high_quality_tracks, moving_objects)
            self.last_stats_report = current_time
    
    def _publish_tracked_objects(self, 
                               tracked_objects: List[TrackedObject3DPython],
                               original_header: Header,
                               processing_time_ms: float,
                               publisher,
                               is_continuous: bool = False) -> None:
        """
        Publish TrackedObjects3D message
        Now handles both detection-driven and continuous publishing
        """
        msg = TrackedObjects3D()
        msg.header = original_header
        msg.total_tracks = len(tracked_objects)
        msg.processing_time_ms = processing_time_ms
        
        high_quality_count = 0
        moving_count = 0
        
        for track in tracked_objects:
            track_msg = TrackedObject3D()
            
            # Basic info
            track_msg.id = track.id
            track_msg.class_id = track.class_id
            track_msg.class_name = track.class_name
            
            # 3D position and motion (may be predicted!)
            track_msg.position_3d = Point(
                x=float(track.position_3d[0]),
                y=float(track.position_3d[1]), 
                z=float(track.position_3d[2])
            )
            
            track_msg.velocity_3d = Vector3(
                x=float(track.velocity_3d[0]),
                y=float(track.velocity_3d[1]),
                z=float(track.velocity_3d[2])
            )
            
            track_msg.acceleration_3d = Vector3(
                x=float(track.acceleration_3d[0]),
                y=float(track.acceleration_3d[1]),
                z=float(track.acceleration_3d[2])
            )
            
            track_msg.speed_3d = float(track.speed_3d)
            
            # 2D bounding box (for visualization) - may be scaled during prediction
            track_msg.x = float(track.position_2d[0])
            track_msg.y = float(track.position_2d[1])
            track_msg.width = float(track.size_2d[0])
            track_msg.height = float(track.size_2d[1])
            
            # Quality metrics (may be reduced during prediction)
            track_msg.confidence = float(track.confidence)
            track_msg.distance_confidence = float(track.distance_confidence)
            track_msg.lidar_points_count = int(track.lidar_points_count)
            track_msg.track_stability = float(track.track_stability)
            
            # Tracking info
            track_msg.track_duration = float(track.track_duration)
            track_msg.total_detections = int(track.total_detections)
            track_msg.is_moving = bool(track.is_moving)
            
            # Prediction - always computed
            track_msg.predicted_position_1s = Point(
                x=float(track.predicted_position_1s[0]),
                y=float(track.predicted_position_1s[1]),
                z=float(track.predicted_position_1s[2])
            )
            
            msg.objects.append(track_msg)
            
            # Count categories
            if track.is_high_quality_track():
                high_quality_count += 1
            if track.is_moving:
                moving_count += 1
        
        msg.high_quality_tracks = high_quality_count
        msg.moving_objects = moving_count
        
        publisher.publish(msg)
        
        # Minimal debug logging for continuous publishing
        if is_continuous and not self.quiet_mode and len(tracked_objects) > 0:
            predicting_count = len([t for t in tracked_objects if t.is_predicting])
            if predicting_count > 0 and self.publishing_stats['continuous_publishes'] % 200 == 0:
                rospy.logdebug(f"Continuous: Published {len(tracked_objects)} tracks ({predicting_count} predicted)")
    
    def _report_enhanced_statistics(self, current_time, all_tracks, high_quality_tracks, moving_objects):
        """Enhanced statistics report with prediction and publishing info"""
        total_time = current_time - self.start_time
        fps = self.frame_count / total_time if total_time > 0 else 0
        avg_processing = self.total_processing_time / self.frame_count if self.frame_count > 0 else 0

        stats = self.tracker.get_stats()
        rain_filtered = self.tracker.prediction_stats.get('rain_filtered', 0)

        rospy.loginfo("=== 3D TRACKING STATISTICS ===")
        rospy.loginfo(f"Frames: {self.frame_count} ({fps:.1f} Hz) | Avg processing: {avg_processing:.1f} ms")
        rospy.loginfo(f"Tracks: {stats['active_tracks']} active, {len(high_quality_tracks)} HQ, {len(moving_objects)} moving")
        rospy.loginfo(f"Predictions: {stats['predicting_tracks']} active, {stats['successful_recoveries']}/{stats['total_predictions']} recovered")

        # Calculate prediction success rate
        if stats['total_predictions'] > 0:
            success_rate = (stats['successful_recoveries'] / stats['total_predictions']) * 100
            rospy.loginfo(f"Prediction success rate: {success_rate:.1f}%")

        # Rain filtering statistics
        if rain_filtered > 0:
            rospy.loginfo(f"Rain/noise filtered: {rain_filtered} detections")

        # Calculate average tracking distance
        if all_tracks:
            avg_distance = np.mean([np.linalg.norm(track.position_3d) for track in all_tracks])
            rospy.loginfo(f"Avg tracking distance: {avg_distance:.1f} m")

        # Show current predicting tracks
        predicting_tracks = [t for t in all_tracks if t.is_predicting]
        if predicting_tracks and len(predicting_tracks) <= 3:
            for track in predicting_tracks:
                rospy.loginfo(f"  Track {track.id} ({track.class_name}): predicting {track.prediction_duration:.2f}s")
        elif len(predicting_tracks) > 3:
            rospy.loginfo(f"  {len(predicting_tracks)} tracks currently predicting")

        rospy.loginfo("==============================")
    
    def run(self):
        """Run the tracking node with continuous publishing"""
        if not self.quiet_mode:
            rospy.loginfo("Python 3D Object Tracking Node running (Press Ctrl+C to stop)")

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            if not self.quiet_mode:
                rospy.loginfo("Python 3D Object Tracking Node shutting down")
        except KeyboardInterrupt:
            if not self.quiet_mode:
                rospy.loginfo("Shutting down gracefully...")


def main():
    """Main entry point"""
    try:
        # Check dependencies
        try:
            import scipy
        except ImportError:
            rospy.logerr("scipy required - install with: pip3 install scipy")
            return

        # Initialize and run node
        node = Tracking3DNode()
        node.run()

    except Exception as e:
        rospy.logerr(f"Failed to start 3D tracking node: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()