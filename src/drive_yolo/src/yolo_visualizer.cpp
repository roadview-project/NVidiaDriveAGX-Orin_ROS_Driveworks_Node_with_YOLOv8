#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <drive_yolo/Detections.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Header.h>

class YoloVisualizer {
public:
    YoloVisualizer() : nh_("~"), frame_count_(0) {
        // Get parameters
        nh_.param<std::string>("output_path", output_path_, "/tmp/yolo_detections.jpg");
        nh_.param<bool>("save_all_frames", save_all_frames_, false);
        nh_.param<int>("save_every_n_frames", save_every_n_frames_, 30); // Save every 30 frames
        nh_.param<bool>("draw_confidence", draw_confidence_, true);
        nh_.param<bool>("draw_class_names", draw_class_names_, true);
        nh_.param<double>("min_confidence_display", min_confidence_display_, 0.3);
        
        ROS_INFO("YOLO Visualizer Configuration:");
        ROS_INFO("  Output path: %s", output_path_.c_str());
        ROS_INFO("  Save all frames: %s", save_all_frames_ ? "YES" : "NO");
        ROS_INFO("  Save every N frames: %d", save_every_n_frames_);
        ROS_INFO("  Draw confidence: %s", draw_confidence_ ? "YES" : "NO");
        ROS_INFO("  Draw class names: %s", draw_class_names_ ? "YES" : "NO");
        ROS_INFO("  Min confidence for display: %.2f", min_confidence_display_);
        
        // Setup synchronized subscribers
        image_sub_.subscribe(nh_, "/camera/image_raw", 10);
        detection_sub_.subscribe(nh_, "/detections", 10);
        
        // Synchronize image and detection messages
        sync_.reset(new Synchronizer(SyncPolicy(10), image_sub_, detection_sub_));
        sync_->registerCallback(boost::bind(&YoloVisualizer::syncCallback, this, _1, _2));
        
        ROS_INFO("YOLO Visualizer started - subscribed to /camera/image_raw and /detections");
        ROS_INFO("Saving visualization to: %s", output_path_.c_str());
    }

private:
    ros::NodeHandle nh_;
    
    // Synchronized subscribers
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<drive_yolo::Detections> detection_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, drive_yolo::Detections> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    std::shared_ptr<Synchronizer> sync_;
    
    // Parameters
    std::string output_path_;
    bool save_all_frames_;
    int save_every_n_frames_;
    bool draw_confidence_;
    bool draw_class_names_;
    double min_confidence_display_;
    
    // State
    int frame_count_;
    
    // Colors for different classes (BGR format for OpenCV)
    std::vector<cv::Scalar> class_colors_ = {
        cv::Scalar(255, 0, 0),     // Blue
        cv::Scalar(0, 255, 0),     // Green  
        cv::Scalar(0, 0, 255),     // Red
        cv::Scalar(255, 255, 0),   // Cyan
        cv::Scalar(255, 0, 255),   // Magenta
        cv::Scalar(0, 255, 255),   // Yellow
        cv::Scalar(128, 0, 128),   // Purple
        cv::Scalar(255, 165, 0),   // Orange
        cv::Scalar(255, 192, 203), // Pink
        cv::Scalar(0, 128, 128),   // Teal
    };
    
    void syncCallback(const sensor_msgs::Image::ConstPtr& image_msg, 
                     const drive_yolo::Detections::ConstPtr& detections_msg) {
        
        frame_count_++;
        
        // Convert ROS image to OpenCV manually (avoiding cv_bridge)
        cv::Mat image = rosImageToCvMat(image_msg);
        
        if (image.empty()) {
            ROS_WARN("Received empty image");
            return;
        }
        
        // Draw detections
        cv::Mat vis_image = image.clone();
        drawDetections(vis_image, detections_msg);
        
        // Add frame info
        drawFrameInfo(vis_image, detections_msg, frame_count_);
        
        // Save image based on configuration
        bool should_save = false;
        std::string save_path = output_path_;
        
        if (save_all_frames_) {
            // Save with frame number
            size_t dot_pos = output_path_.find_last_of('.');
            if (dot_pos != std::string::npos) {
                save_path = output_path_.substr(0, dot_pos) + "_" + 
                           std::to_string(frame_count_) + output_path_.substr(dot_pos);
            } else {
                save_path = output_path_ + "_" + std::to_string(frame_count_);
            }
            should_save = true;
        } else if (frame_count_ % save_every_n_frames_ == 0) {
            should_save = true;
        }
        
        if (should_save) {
            if (cv::imwrite(save_path, vis_image)) {
                ROS_INFO("Frame %d: Saved visualization with %d detections to %s", 
                        frame_count_, static_cast<int>(detections_msg->detections.size()), save_path.c_str());
            } else {
                ROS_ERROR("Failed to save image to %s", save_path.c_str());
            }
        }
        
        // Print detection summary
        if (!detections_msg->detections.empty()) {
            ROS_INFO("Frame %d: %d detections, inference: %.1f ms", 
                    frame_count_, static_cast<int>(detections_msg->detections.size()), 
                    detections_msg->inference_time_ms);
        }
    }
    
    void drawDetections(cv::Mat& image, const drive_yolo::Detections::ConstPtr& detections) {
        for (const auto& detection : detections->detections) {
            // Skip low confidence detections
            if (detection.confidence < min_confidence_display_) {
                continue;
            }
            
            // Calculate bounding box corners
            int x1 = static_cast<int>(detection.x - detection.width / 2.0f);
            int y1 = static_cast<int>(detection.y - detection.height / 2.0f);
            int x2 = static_cast<int>(detection.x + detection.width / 2.0f);
            int y2 = static_cast<int>(detection.y + detection.height / 2.0f);
            
            // Clamp to image bounds
            x1 = std::max(0, std::min(x1, image.cols - 1));
            y1 = std::max(0, std::min(y1, image.rows - 1));
            x2 = std::max(0, std::min(x2, image.cols - 1));
            y2 = std::max(0, std::min(y2, image.rows - 1));
            
            // Get color for this class
            cv::Scalar color = getClassColor(detection.class_id);
            
            // Draw bounding box
            cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
            
            // Prepare label text
            std::string label;
            if (draw_class_names_ && draw_confidence_) {
                label = detection.class_name + " " + 
                       std::to_string(static_cast<int>(detection.confidence * 100)) + "%";
            } else if (draw_class_names_) {
                label = detection.class_name;
            } else if (draw_confidence_) {
                label = std::to_string(static_cast<int>(detection.confidence * 100)) + "%";
            } else {
                label = "ID:" + std::to_string(detection.class_id);
            }
            
            // Draw label background
            int font_face = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.6;
            int thickness = 1;
            int baseline = 0;
            
            cv::Size text_size = cv::getTextSize(label, font_face, font_scale, thickness, &baseline);
            
            // Position label above the box, or below if there's no space
            int label_y = (y1 - text_size.height - 10 > 0) ? y1 - 10 : y2 + text_size.height + 10;
            
            // Draw label background
            cv::rectangle(image, 
                         cv::Point(x1, label_y - text_size.height - 5),
                         cv::Point(x1 + text_size.width + 10, label_y + 5),
                         color, -1);
            
            // Draw label text
            cv::putText(image, label, 
                       cv::Point(x1 + 5, label_y - 5),
                       font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
            
            // Draw center point
            cv::circle(image, cv::Point(static_cast<int>(detection.x), static_cast<int>(detection.y)), 
                      3, color, -1);
        }
    }
    
    void drawFrameInfo(cv::Mat& image, const drive_yolo::Detections::ConstPtr& detections, int frame_num) {
        // Create info text
        std::string info = "Frame: " + std::to_string(frame_num) + 
                          " | Detections: " + std::to_string(detections->detections.size()) +
                          " | Inference: " + std::to_string(static_cast<int>(detections->inference_time_ms)) + "ms";
        
        // Add timestamp
        auto stamp = detections->header.stamp;
        info += " | Time: " + std::to_string(stamp.sec) + "." + std::to_string(stamp.nsec / 1000000);
        
        // Draw info background (top-left corner)
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.7;
        int thickness = 2;
        int baseline = 0;
        
        cv::Size text_size = cv::getTextSize(info, font_face, font_scale, thickness, &baseline);
        
        // Dark background for readability
        cv::rectangle(image, 
                     cv::Point(5, 5),
                     cv::Point(text_size.width + 15, text_size.height + 15),
                     cv::Scalar(0, 0, 0), -1);
        
        // White text
        cv::putText(image, info, cv::Point(10, text_size.height + 10),
                   font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
    }
    
    cv::Scalar getClassColor(int class_id) {
        return class_colors_[class_id % class_colors_.size()];
    }
    
    // Manual ROS Image to OpenCV conversion (avoiding cv_bridge)
    cv::Mat rosImageToCvMat(const sensor_msgs::Image::ConstPtr& msg) {
        cv::Mat image;
        
        if (msg->encoding == "bgr8") {
            image = cv::Mat(msg->height, msg->width, CV_8UC3, 
                           const_cast<unsigned char*>(msg->data.data()), msg->step);
            image = image.clone(); // Make a copy since we're using const data
            
        } else if (msg->encoding == "rgb8") {
            cv::Mat temp(msg->height, msg->width, CV_8UC3, 
                        const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::cvtColor(temp, image, cv::COLOR_RGB2BGR);
            
        } else if (msg->encoding == "mono8") {
            cv::Mat temp(msg->height, msg->width, CV_8UC1, 
                        const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::cvtColor(temp, image, cv::COLOR_GRAY2BGR);
            
        } else if (msg->encoding == "rgba8") {
            cv::Mat temp(msg->height, msg->width, CV_8UC4, 
                        const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::cvtColor(temp, image, cv::COLOR_RGBA2BGR);
            
        } else {
            ROS_ERROR("Unsupported image encoding: %s", msg->encoding.c_str());
            return cv::Mat();
        }
        
        return image;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_visualizer");
    
    try {
        YoloVisualizer visualizer;
        
        ROS_INFO("YOLO Visualizer running... Press Ctrl+C to stop");
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Visualizer failed: %s", e.what());
        return -1;
    }
    
    ROS_INFO("YOLO Visualizer shutting down");
    return 0;
}