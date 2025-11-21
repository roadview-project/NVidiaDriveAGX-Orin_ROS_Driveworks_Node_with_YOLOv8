#include "ros/ros.h"
#include "drive_yolo/yolo_infer.hpp"
#include "sensor_msgs/Image.h"
#include "drive_yolo/Detection.h"
#include "drive_yolo/Detections.h"
#include <opencv2/opencv.hpp>
#include <exception>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

struct ImageFrame {
    cv::Mat image;
    uint64_t timestamp;
    ros::Time ros_time;
};

class YoloNode {
public:
    YoloNode() : nh_("~"), frame_count_(0), detection_count_(0), total_inference_time_(0.0), running_(false) {
        // Get model path from parameter - try both private and global
        std::string model_path;
        if (!nh_.getParam("model_path", model_path)) {
            ros::NodeHandle global_nh;
            if (!global_nh.getParam("model_path", model_path)) {
                model_path = "/models/yolov4s.onnx";
            }
        }
        nh_.param<double>("vehicle_confidence_threshold", vehicle_confidence_threshold_, 0.5);
        nh_.param<double>("pedestrian_confidence_threshold", pedestrian_confidence_threshold_, 0.2);

        ROS_INFO("Per-class thresholds - Vehicles (id=0): %.2f, Pedestrians (id=1): %.2f",
         vehicle_confidence_threshold_, pedestrian_confidence_threshold_);
        // Get camera mode
        bool use_driveworks_camera;
        nh_.param<bool>("use_driveworks_camera", use_driveworks_camera, false);
        
        // Get confidence thresholds
        nh_.param<double>("confidence_threshold", confidence_threshold_, 0.7);
        nh_.param<double>("detection_threshold", detection_threshold_, 0.5);
        nh_.param<int>("ignore_hood_pixels", ignore_hood_pixels_, 400);
        nh_.param<bool>("enable_hood_filtering", enable_hood_filtering_, true);

        nh_.param<bool>("quiet_mode", quiet_mode_, false);
    
        // Only show config if NOT in quiet mode
        if (!quiet_mode_) {
            ROS_INFO("Loading YOLO model: %s", model_path.c_str());
            ROS_INFO("Thresholds - Detection: %.2f, Publishing: %.2f", detection_threshold_, confidence_threshold_);
            ROS_INFO("DriveWorks camera: %s", use_driveworks_camera ? "ENABLED" : "DISABLED");
            ROS_INFO("Quiet mode: %s", quiet_mode_ ? "ENABLED" : "DISABLED");
        } else {
            // In quiet mode, only show essential startup info
            ROS_INFO("YOLO starting in QUIET MODE - model: %s", model_path.c_str());
        }
        
        // Get quiet mode setting
        nh_.param<bool>("quiet_mode", quiet_mode_, false);
        
        ROS_INFO("Loading YOLO model: %s", model_path.c_str());
        ROS_INFO("Thresholds - Detection: %.2f, Publishing: %.2f", detection_threshold_, confidence_threshold_);
        ROS_INFO("DriveWorks camera: %s", use_driveworks_camera ? "ENABLED" : "DISABLED");
        ROS_INFO("Quiet mode: %s", quiet_mode_ ? "ENABLED" : "DISABLED");
        
        try {
            // Initialize YOLO
            yolo_ = std::make_unique<YoloInfer>(model_path);
            yolo_->printModelInfo();
            ROS_INFO("YOLO ready!");
            
            // Publishers - only the essentials
            detections_pub_ = nh_.advertise<drive_yolo::Detections>("/detections", 50);
            raw_image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image_raw", 5);
            running_ = true;
            // Start processing thread BEFORE camera thread
            processing_thread_ = std::thread(&YoloNode::processingLoop, this);
            if (use_driveworks_camera) {
#ifdef USE_DRIVEWORKS
                std::string camera_params;
                nh_.param<std::string>("camera_params", camera_params, 
                    "camera-name=F008A120RM0AV2,interface=csi-ab,link=1,output-format=processed");
                
                camera_ = std::make_unique<DriveWorksCamera>();
                if (!camera_->initialize(camera_params) || !camera_->start()) {
                    throw std::runtime_error("DriveWorks camera init failed");
                }
                
                ROS_INFO("DriveWorks camera started");
                camera_thread_ = std::thread(&YoloNode::cameraLoop, this);
#else
                ROS_ERROR("DriveWorks not compiled! Using ROS mode");
                use_driveworks_camera = false;
#endif
            }
            
            
            if (!use_driveworks_camera) {
                std::string image_topic;
                nh_.param<std::string>("image_topic", image_topic, "/camera/image_input");
                image_sub_ = nh_.subscribe(image_topic, 10, &YoloNode::imageCallback, this);
                ROS_INFO("Subscribed to: %s", image_topic.c_str());
            }
            
            ROS_INFO("=== DRIVE YOLO READY ===");
            ROS_INFO("Publishing: /detections, /camera/image_raw");
            
            // Start timing
            start_time_ = std::chrono::high_resolution_clock::now();
            last_stats_report_ = start_time_;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Init failed: %s", e.what());
            throw;
        }
    }
    

    ~YoloNode() {
    ROS_INFO("Starting YoloNode shutdown...");
    
    // Step 1: Signal all threads to stop
    running_ = false;
    
    // Step 2: Stop camera first (stop new frames from coming in)
#ifdef USE_DRIVEWORKS
    if (camera_) {
        ROS_INFO("Stopping DriveWorks camera...");
        try {
            camera_->stop();
        } catch (const std::exception& e) {
            ROS_WARN("Exception stopping camera: %s", e.what());
        } catch (...) {
            ROS_WARN("Unknown exception stopping camera");
        }
    }
#endif
    
    // Step 3: Wake up and join processing thread first
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // Clear the queue
        while (!image_queue_.empty()) {
            image_queue_.pop();
        }
        // Add dummy frame to wake up processing thread
        ImageFrame dummy_frame;
        dummy_frame.image = cv::Mat(); // Empty image signals shutdown
        image_queue_.push(dummy_frame);
    }
    queue_condition_.notify_all();
    
    if (processing_thread_.joinable()) {
        ROS_INFO("Joining processing thread...");
        if (!joinThreadWithTimeout(processing_thread_, 2000)) { // 2 second timeout
            ROS_WARN("Processing thread didn't join gracefully");
        }
    }
    
    // Step 4: Join camera thread
    if (camera_thread_.joinable()) {
        ROS_INFO("Joining camera thread...");
        if (!joinThreadWithTimeout(camera_thread_, 2000)) { // 2 second timeout
            ROS_WARN("Camera thread didn't join gracefully");
        }
    }
    
    // Step 5: Cleanup DriveWorks (after threads are stopped)
#ifdef USE_DRIVEWORKS
    if (camera_) {
        ROS_INFO("Cleaning up DriveWorks camera...");
        try {
            camera_.reset(); // This calls the DriveWorksCamera destructor
        } catch (const std::exception& e) {
            ROS_WARN("Exception during camera cleanup: %s", e.what());
        } catch (...) {
            ROS_WARN("Unknown exception during camera cleanup");
        }
    }
#endif
    
    // Step 6: Cleanup YOLO (CUDA resources)
    if (yolo_) {
        ROS_INFO("Cleaning up YOLO inference...");
        try {
            yolo_.reset(); // This calls the YoloInfer destructor
        } catch (const std::exception& e) {
            ROS_WARN("Exception during YOLO cleanup: %s", e.what());
        } catch (...) {
            ROS_WARN("Unknown exception during YOLO cleanup");
        }
    }
    
    // Step 7: Give CUDA a moment to finish
    try {
        cudaDeviceSynchronize();
        ROS_INFO("CUDA synchronized");
    } catch (...) {
        ROS_WARN("Exception during CUDA sync");
    }
    
    ROS_INFO("YoloNode shutdown complete");
}

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher detections_pub_;
    ros::Publisher raw_image_pub_;
    
    std::unique_ptr<YoloInfer> yolo_;
#ifdef USE_DRIVEWORKS    
    std::unique_ptr<DriveWorksCamera> camera_;
#endif

    // Configuration
    double vehicle_confidence_threshold_;     // For class_id 0 (vehicles)
    double pedestrian_confidence_threshold_;  // For class_id 1 (pedestrians)

    double confidence_threshold_;
    double detection_threshold_;
    bool quiet_mode_;
    int ignore_hood_pixels_;
    bool enable_hood_filtering_;
    
    // Threading
    std::thread camera_thread_;
    std::thread processing_thread_;
    std::atomic<bool> running_;
    
    // Frame queue for decoupling camera from processing
    std::queue<ImageFrame> image_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    
    // Performance tracking
    int frame_count_;
    int detection_count_;
    double total_inference_time_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_stats_report_;
    
    // System boot time for timestamp conversion
    std::chrono::steady_clock::time_point boot_time_;
    bool boot_time_initialized_ = false;
    
    void initBootTime() {
        if (!boot_time_initialized_) {
            // Calculate approximate boot time
            auto now_steady = std::chrono::steady_clock::now();
            auto now_system = std::chrono::system_clock::now();
            auto uptime_duration = now_steady.time_since_epoch();
            boot_time_ = now_steady - uptime_duration;
            boot_time_initialized_ = true;
            
            ROS_INFO("Boot time initialized for timestamp conversion");
        }
    }
    
    ros::Time convertDeviceTimestamp(uint64_t device_timestamp_us) {
        if (!boot_time_initialized_) {
            initBootTime();
        }
        
        // Convert device timestamp (microseconds since boot) to ROS time
        if (device_timestamp_us == 0) {
            return ros::Time::now();
        }
        
        // Method 1: Convert device time to epoch time
        auto device_duration = std::chrono::microseconds(device_timestamp_us);
        auto epoch_time = std::chrono::system_clock::now() - 
                         (std::chrono::steady_clock::now().time_since_epoch() - device_duration);
        
        auto time_t_value = std::chrono::system_clock::to_time_t(epoch_time);
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
            epoch_time.time_since_epoch()).count() % 1000000;
        
        return ros::Time(time_t_value, microseconds * 1000); // Convert microseconds to nanoseconds
    }
    bool joinThreadWithTimeout(std::thread& thread, int timeout_ms) {
        if (!thread.joinable()) return true;
        
        auto start = std::chrono::steady_clock::now();
        
        while (thread.joinable()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            
            if (elapsed > timeout_ms) {
                ROS_WARN("Thread join timeout after %d ms", timeout_ms);
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // Try to join with a short timeout
            if (thread.joinable()) {
                try {
                    thread.join();
                    return true;
                } catch (...) {
                    // Thread might still be running
                    continue;
                }
            }
        }
        return true;
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        cv::Mat image = convertRosImageToCvMat(msg);
        if (image.empty()) {
            return;
        }
        
        // Republish raw image
        raw_image_pub_.publish(*msg);
        
        // Add to processing queue
        ImageFrame frame;
        frame.image = image;
        frame.timestamp = 0; // ROS timestamp
        frame.ros_time = msg->header.stamp;
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (image_queue_.size() < 10) { // Limit queue size
                image_queue_.push(frame);
                queue_condition_.notify_one();
            } 
        }
    }
    
    void cameraLoop() {
#ifdef USE_DRIVEWORKS
    if (!quiet_mode_) {
        ROS_INFO("Camera thread started");
    }
    ImageFrame frame;
    
    while (running_ && ros::ok()) {
        try {
            if (camera_ && camera_->getNextFrame(frame.image, frame.timestamp)) {
                // Convert timestamp
                frame.ros_time =  ros::Time::now(); //convertDeviceTimestamp(frame.timestamp);
                
                // Publish raw image immediately
                publishRawImage(frame.image, frame.ros_time);
                
                // Add to processing queue (non-blocking)
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    if (image_queue_.size() < 3) {
                        image_queue_.push(frame);
                        queue_condition_.notify_one();
                    }
                }
            } else if (!running_) {
                break; // Normal shutdown
            }
        } catch (const std::exception& e) {
            if (!quiet_mode_) {
                ROS_WARN("Exception in camera loop: %s", e.what());
            }
            if (!running_) break;
        } catch (...) {
            if (!quiet_mode_) {
                ROS_WARN("Unknown exception in camera loop");
            }
            if (!running_) break;
        }
    }
    
    if (!quiet_mode_) {
        ROS_INFO("Camera thread exiting");
    }
#endif
}

void processingLoop() {
    if (!quiet_mode_) {
        ROS_INFO("Processing thread started");
    }
    
    while (running_) {
        ImageFrame frame;
        
        // Wait for new frame
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_condition_.wait(lock, [this] { return !image_queue_.empty() || !running_; });
            
            if (!running_) {
                if (!quiet_mode_) {
                    ROS_INFO("Processing thread received shutdown signal");
                }
                break;
            }
            
            if (image_queue_.empty()) continue;
            
            frame = image_queue_.front();
            image_queue_.pop();
        }
        
        // Check for shutdown signal (empty image)
        if (frame.image.empty() && !running_) {
            if (!quiet_mode_) {
                ROS_INFO("Processing thread received empty frame shutdown signal");
            }
            break;
        }
        
        if (!frame.image.empty()) {
            try {
                processImage(frame.image, frame.ros_time);
            } catch (const std::exception& e) {
                if (!quiet_mode_) {
                    ROS_WARN("Exception in processImage: %s", e.what());
                }
            } catch (...) {
                if (!quiet_mode_) {
                    ROS_WARN("Unknown exception in processImage");
                }
            }
        }
    }
    
    if (!quiet_mode_) {
        ROS_INFO("Processing thread exiting");
    }
}



    
    cv::Mat convertRosImageToCvMat(const sensor_msgs::Image::ConstPtr& msg) {
    cv::Mat image;
    
    // Debug info
    ROS_DEBUG("Converting ROS image: %dx%d, encoding=%s, step=%d", 
              msg->width, msg->height, msg->encoding.c_str(), msg->step);
    
    if (msg->encoding == "bgr8") {
        // Check if step matches expected value
        uint32_t expected_step = msg->width * 3;
        if (msg->step == expected_step) {
            // No padding, can create matrix directly
            image = cv::Mat(msg->height, msg->width, CV_8UC3, 
                           const_cast<unsigned char*>(msg->data.data()), msg->step);
        } else {
            // Has padding, need to copy without padding
            image = cv::Mat(msg->height, msg->width, CV_8UC3);
            for (uint32_t row = 0; row < msg->height; ++row) {
                const uint8_t* src_row = msg->data.data() + row * msg->step;
                uint8_t* dst_row = image.ptr<uint8_t>(row);
                memcpy(dst_row, src_row, msg->width * 3);
            }
        }
        image = image.clone(); // Ensure we own the data
        
    } else if (msg->encoding == "rgb8") {
        // Similar handling for RGB
        cv::Mat temp_image;
        uint32_t expected_step = msg->width * 3;
        if (msg->step == expected_step) {
            temp_image = cv::Mat(msg->height, msg->width, CV_8UC3, 
                               const_cast<unsigned char*>(msg->data.data()), msg->step);
        } else {
            temp_image = cv::Mat(msg->height, msg->width, CV_8UC3);
            for (uint32_t row = 0; row < msg->height; ++row) {
                const uint8_t* src_row = msg->data.data() + row * msg->step;
                uint8_t* dst_row = temp_image.ptr<uint8_t>(row);
                memcpy(dst_row, src_row, msg->width * 3);
            }
        }
        cv::cvtColor(temp_image, image, cv::COLOR_RGB2BGR);
        
    } else if (msg->encoding == "bgra8" || msg->encoding == "rgba8") {
        // Handle 4-channel images
        cv::Mat temp_image;
        uint32_t expected_step = msg->width * 4;
        if (msg->step == expected_step) {
            temp_image = cv::Mat(msg->height, msg->width, CV_8UC4, 
                               const_cast<unsigned char*>(msg->data.data()), msg->step);
        } else {
            temp_image = cv::Mat(msg->height, msg->width, CV_8UC4);
            for (uint32_t row = 0; row < msg->height; ++row) {
                const uint8_t* src_row = msg->data.data() + row * msg->step;
                uint8_t* dst_row = temp_image.ptr<uint8_t>(row);
                memcpy(dst_row, src_row, msg->width * 4);
            }
        }
        
        if (msg->encoding == "rgba8") {
            cv::cvtColor(temp_image, image, cv::COLOR_RGBA2BGR);
        } else {
            cv::cvtColor(temp_image, image, cv::COLOR_BGRA2BGR);
        }
        
    } else if (msg->encoding == "mono8") {
        uint32_t expected_step = msg->width * 1;
        if (msg->step == expected_step) {
            image = cv::Mat(msg->height, msg->width, CV_8UC1, 
                           const_cast<unsigned char*>(msg->data.data()), msg->step);
        } else {
            image = cv::Mat(msg->height, msg->width, CV_8UC1);
            for (uint32_t row = 0; row < msg->height; ++row) {
                const uint8_t* src_row = msg->data.data() + row * msg->step;
                uint8_t* dst_row = image.ptr<uint8_t>(row);
                memcpy(dst_row, src_row, msg->width);
            }
        }
        image = image.clone();
        
    } else {
        ROS_ERROR("Unsupported image encoding: %s", msg->encoding.c_str());
        return cv::Mat();
    }
    
    if (image.empty()) {
        ROS_ERROR("Failed to convert ROS image to OpenCV Mat");
    }
    
    return image;
}
    void processImage(const cv::Mat& image, const ros::Time& timestamp) {
    // Run YOLO inference
    auto inference_start = std::chrono::high_resolution_clock::now();
    std::vector<Detection> detections = yolo_->detect(image, detection_threshold_, 0.4f);
    auto inference_end = std::chrono::high_resolution_clock::now();
    
    double inference_time = std::chrono::duration_cast<std::chrono::microseconds>(inference_end - inference_start).count() / 1000.0;
    
    // Update stats
    frame_count_++;
    total_inference_time_ += inference_time;
    
    // Filter and publish high confidence detections
    std::vector<Detection> high_conf_detections;
    for (const auto& det : detections) {
        // Per-class confidence filtering
        float required_confidence = confidence_threshold_;  // Default threshold

        // Apply class-specific thresholds
        if (det.class_id == 0) {
            // Vehicle (car, truck, bus, etc.)
            required_confidence = vehicle_confidence_threshold_;
        } else if (det.class_id == 1) {
            // Pedestrian
            required_confidence = pedestrian_confidence_threshold_;
        }
        // All other classes use the default confidence_threshold_

        // Check against the appropriate threshold
        if (det.confidence < required_confidence) {
            if (!quiet_mode_) {
                ROS_DEBUG("Filtered %s detection: %.2f%% < %.2f%% threshold",
                         getClassName(det.class_id).c_str(), det.confidence * 100,
                         required_confidence * 100);
            }
            continue;
        }

        // NEW: Hood filtering - skip detections in lower portion of image
        if (enable_hood_filtering_) {
            float detection_bottom = det.y + (det.height / 2.0f);
            float hood_threshold = image.rows - ignore_hood_pixels_; // e.g., 2148 - 400 = 1748
            
            if (detection_bottom > hood_threshold) {
                if (!quiet_mode_) {
                    ROS_DEBUG("Filtered hood detection: %s at y=%.0f (threshold=%.0f)", 
                             getClassName(det.class_id).c_str(), detection_bottom, hood_threshold);
                }
                continue; // Skip this detection
            }
        }
        
        high_conf_detections.push_back(det);
    }
    if (!high_conf_detections.empty()) {
        publishDetections(high_conf_detections, inference_time, timestamp);
        detection_count_++;
        
        // ENHANCED: Only log detections if not in quiet mode
        if (!quiet_mode_) {
            ROS_INFO("Frame %d: %.1f ms, %zu detections", frame_count_, inference_time, high_conf_detections.size());
            for (size_t i = 0; i < high_conf_detections.size() && i < 3; ++i) {
                const auto& det = high_conf_detections[i];
                ROS_INFO("  %s: %.1f%% [%.0f,%.0f,%.0fx%.0f]", 
                         getClassName(det.class_id).c_str(), det.confidence * 100,
                         det.x, det.y, det.width, det.height);
            }
            if (high_conf_detections.size() > 3) {
                ROS_INFO("  ... and %zu more", high_conf_detections.size() - 3);
            }
        }
    }
    
    // ENHANCED: Report stats based on quiet mode
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_since_report = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_stats_report_).count();
    
    int stats_interval = quiet_mode_ ? 60 : 10;  // 60 seconds in quiet mode, 10 seconds normally
    
    if (time_since_report >= stats_interval) {
        auto total_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
        double overall_fps = static_cast<double>(frame_count_) / total_time;
        double detection_fps = static_cast<double>(detection_count_) / total_time;
        double avg_inference = total_inference_time_ / frame_count_;
        
        if (!quiet_mode_) {
            // Normal verbose stats
            double max_theoretical_fps = 1000.0 / avg_inference;
            ROS_INFO("=== STATS ===");
            ROS_INFO("Frames: %d (%.1f Hz), Detections: %d (%.1f Hz)", 
                     frame_count_, overall_fps, detection_count_, detection_fps);
            ROS_INFO("Inference: %.1f ms avg, %.1f Hz max theoretical", avg_inference, max_theoretical_fps);
            ROS_INFO("Queue size: %zu", image_queue_.size());
            ROS_INFO("=============");
        } else {
            // Quiet mode: minimal stats
            ROS_INFO("Stats: %d frames (%.1fHz), %d detections (%.1fHz), %.1fms avg", 
                     frame_count_, overall_fps, detection_count_, detection_fps, avg_inference);
        }
        
        last_stats_report_ = current_time;
    }
}
    
    
    void publishDetections(const std::vector<Detection>& detections, double inference_time_ms, const ros::Time& timestamp) {
        drive_yolo::Detections msg;
        msg.header.stamp = timestamp; // Use corrected timestamp
        msg.header.frame_id = "camera";
        msg.inference_time_ms = inference_time_ms;
        msg.total_detections = detections.size();
        
        for (const auto& det : detections) {
            drive_yolo::Detection det_msg;
            det_msg.x = det.x;
            det_msg.y = det.y;
            det_msg.width = det.width;
            det_msg.height = det.height;
            det_msg.confidence = det.confidence;
            det_msg.class_id = 2; //det.class_id;
            det_msg.class_name = getClassName(det.class_id);
            
            msg.detections.push_back(det_msg);
        }
        
        detections_pub_.publish(msg);
    }

    void publishRawImage(const cv::Mat& image, const ros::Time& timestamp) {
    if (image.empty()) {
        ROS_WARN("Trying to publish empty image");
        return;
    }
    
    sensor_msgs::Image img_msg;
    img_msg.header.stamp = timestamp;
    img_msg.header.frame_id = "camera";
    img_msg.height = image.rows;
    img_msg.width = image.cols;
    
    // Determine encoding and step based on image type
    if (image.channels() == 3) {
        img_msg.encoding = "bgr8";
        img_msg.step = image.cols * 3;  // 3 bytes per pixel for BGR
    } else if (image.channels() == 4) {
        img_msg.encoding = "bgra8"; 
        img_msg.step = image.cols * 4;  // 4 bytes per pixel for BGRA
    } else if (image.channels() == 1) {
        img_msg.encoding = "mono8";
        img_msg.step = image.cols * 1;  // 1 byte per pixel for grayscale
    } else {
        ROS_ERROR("Unsupported image format: %d channels", image.channels());
        return;
    }
    
    img_msg.is_bigendian = false;
    
    // Check if image data is continuous (no padding between rows)
    if (image.isContinuous()) {
        // Image data is continuous, can copy directly
        size_t data_size = img_msg.step * img_msg.height;
        img_msg.data.resize(data_size);
        memcpy(img_msg.data.data(), image.data, data_size);
        
        ROS_DEBUG("Published continuous image: %dx%d, %d channels, step=%d", 
                  img_msg.width, img_msg.height, image.channels(), img_msg.step);
    } else {
        // Image has padding, need to copy row by row
        img_msg.data.resize(img_msg.step * img_msg.height);
        
        for (int row = 0; row < image.rows; ++row) {
            const uint8_t* src_row = image.ptr<uint8_t>(row);
            uint8_t* dst_row = img_msg.data.data() + row * img_msg.step;
            memcpy(dst_row, src_row, img_msg.step);
        }
        
        ROS_DEBUG("Published non-continuous image: %dx%d, %d channels, step=%d", 
                  img_msg.width, img_msg.height, image.channels(), img_msg.step);
    }
    
    raw_image_pub_.publish(img_msg);
}
    std::string getClassName(int class_id) {
        /*static const std::vector<std::string> class_names = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
            "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
            "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
            "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
            "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
            "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush"
        };*/ //THESE ARE COCO STD
        static const std::vector<std::string> class_names = { //THIS ARE NUSCENES STD
            "vehicle", "pedestrian", "bicycle", "motorcycle"
        };

        if (class_id >= 0 && class_id < static_cast<int>(class_names.size())) {
            return class_names[class_id];
        }
        return "unknown";
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_yolo_node");
    
    try {
        YoloNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Node failed: %s", e.what());
        return -1;
    }
    
    return 0;
}
