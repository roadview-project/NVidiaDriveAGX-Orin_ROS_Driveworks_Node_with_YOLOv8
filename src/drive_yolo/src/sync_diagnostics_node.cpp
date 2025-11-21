/**
 * Synchronization Diagnostics Node
 *
 * This node measures and reports timing differences between camera detections
 * and LiDAR pointcloud data to help diagnose synchronization issues.
 *
 * Features:
 * - Measures timestamp differences between camera and LiDAR
 * - Maintains configurable queue to buffer messages
 * - Reports statistics (mean, std dev, min, max delays)
 * - Publishes diagnostics for visualization
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <drive_yolo/Detections.h>
#include <std_msgs/Float64.h>
#include <deque>
#include <chrono>
#include <numeric>
#include <cmath>

class SyncDiagnostics
{
public:
    SyncDiagnostics() : nh_("~")
    {
        // Load parameters
        nh_.param<int>("queue_size", queue_size_, 50);
        nh_.param<double>("report_interval", report_interval_, 2.0);  // Report every 2 seconds
        nh_.param<std::string>("camera_topic", camera_topic_, "/detections");
        nh_.param<std::string>("lidar_topic", lidar_topic_, "/invz_summation_reflection");

        // Subscribers
        camera_sub_ = nh_.subscribe(camera_topic_, 10, &SyncDiagnostics::cameraCallback, this);
        lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &SyncDiagnostics::lidarCallback, this);

        // Publishers for diagnostics
        delay_pub_ = nh_.advertise<std_msgs::Float64>("/sync_diagnostics/camera_lidar_delay", 10);

        // Timer for periodic reporting
        report_timer_ = nh_.createTimer(ros::Duration(report_interval_),
                                        &SyncDiagnostics::reportStatistics, this);

        ROS_INFO("=== SYNC DIAGNOSTICS NODE ===");
        ROS_INFO("Camera topic: %s", camera_topic_.c_str());
        ROS_INFO("LiDAR topic: %s", lidar_topic_.c_str());
        ROS_INFO("Queue size: %d", queue_size_);
        ROS_INFO("Report interval: %.1f sec", report_interval_);
        ROS_INFO("=============================");
    }

private:
    struct TimestampPair
    {
        ros::Time camera_stamp;
        ros::Time lidar_stamp;
        ros::Time arrival_time;
        double delay_ms;
    };

    void cameraCallback(const drive_yolo::Detections::ConstPtr& msg)
    {
        last_camera_stamp_ = msg->header.stamp;
        last_camera_arrival_ = ros::Time::now();
        camera_count_++;

        // If we have a recent LiDAR message, compute delay
        if (!last_lidar_stamp_.isZero())
        {
            double delay = (last_camera_stamp_ - last_lidar_stamp_).toSec() * 1000.0;  // ms

            TimestampPair pair;
            pair.camera_stamp = last_camera_stamp_;
            pair.lidar_stamp = last_lidar_stamp_;
            pair.arrival_time = ros::Time::now();
            pair.delay_ms = delay;

            addDelay(pair);

            // Publish current delay
            std_msgs::Float64 delay_msg;
            delay_msg.data = delay;
            delay_pub_.publish(delay_msg);
        }
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        last_lidar_stamp_ = msg->header.stamp;
        last_lidar_arrival_ = ros::Time::now();
        lidar_count_++;

        // If we have a recent camera message, compute delay
        if (!last_camera_stamp_.isZero())
        {
            double delay = (last_camera_stamp_ - last_lidar_stamp_).toSec() * 1000.0;  // ms

            TimestampPair pair;
            pair.camera_stamp = last_camera_stamp_;
            pair.lidar_stamp = last_lidar_stamp_;
            pair.arrival_time = ros::Time::now();
            pair.delay_ms = delay;

            addDelay(pair);

            // Publish current delay
            std_msgs::Float64 delay_msg;
            delay_msg.data = delay;
            delay_pub_.publish(delay_msg);
        }
    }

    void addDelay(const TimestampPair& pair)
    {
        delay_queue_.push_back(pair);

        // Maintain queue size
        while (delay_queue_.size() > static_cast<size_t>(queue_size_))
        {
            delay_queue_.pop_front();
        }
    }

    void reportStatistics(const ros::TimerEvent&)
    {
        if (delay_queue_.empty())
        {
            ROS_WARN("No timing data collected yet. Camera msgs: %d, LiDAR msgs: %d",
                     camera_count_, lidar_count_);
            return;
        }

        // Calculate statistics
        std::vector<double> delays;
        delays.reserve(delay_queue_.size());
        for (const auto& pair : delay_queue_)
        {
            delays.push_back(pair.delay_ms);
        }

        double mean = std::accumulate(delays.begin(), delays.end(), 0.0) / delays.size();

        double variance = 0.0;
        for (double delay : delays)
        {
            variance += (delay - mean) * (delay - mean);
        }
        variance /= delays.size();
        double std_dev = std::sqrt(variance);

        auto minmax = std::minmax_element(delays.begin(), delays.end());
        double min_delay = *minmax.first;
        double max_delay = *minmax.second;

        // Calculate median
        std::vector<double> sorted_delays = delays;
        std::sort(sorted_delays.begin(), sorted_delays.end());
        double median = sorted_delays[sorted_delays.size() / 2];

        ROS_INFO("=== SYNC DIAGNOSTICS REPORT ===");
        ROS_INFO("Messages received: Camera=%d, LiDAR=%d", camera_count_, lidar_count_);
        ROS_INFO("Queue size: %zu/%d samples", delay_queue_.size(), queue_size_);
        ROS_INFO("Camera-LiDAR timestamp offset (positive = camera ahead):");
        ROS_INFO("  Mean:   %+7.2f ms", mean);
        ROS_INFO("  Median: %+7.2f ms", median);
        ROS_INFO("  StdDev:  %6.2f ms", std_dev);
        ROS_INFO("  Min:    %+7.2f ms", min_delay);
        ROS_INFO("  Max:    %+7.2f ms", max_delay);

        // Provide recommendation based on mean delay
        if (std::abs(mean) > 50.0)
        {
            int frames_offset = static_cast<int>(std::round(mean / 33.33));  // Assuming ~30 Hz
            ROS_WARN("Large synchronization offset detected!");
            ROS_WARN("Recommendation: Set time_offset_ms = %.1f in fusion config", mean);
            ROS_WARN("Or use frame_offset = %d (at 30 Hz)", frames_offset);
        }
        else
        {
            ROS_INFO("Synchronization looks good (offset < 50 ms)");
        }

        // Show most recent samples
        ROS_INFO("Recent samples (last 5):");
        int start = std::max(0, static_cast<int>(delay_queue_.size()) - 5);
        for (size_t i = start; i < delay_queue_.size(); i++)
        {
            const auto& pair = delay_queue_[i];
            ROS_INFO("  [%zu] Camera: %.3f, LiDAR: %.3f, Delay: %+7.2f ms",
                     i, pair.camera_stamp.toSec(), pair.lidar_stamp.toSec(), pair.delay_ms);
        }

        ROS_INFO("===============================");
    }

    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher delay_pub_;
    ros::Timer report_timer_;

    std::string camera_topic_;
    std::string lidar_topic_;
    int queue_size_;
    double report_interval_;

    ros::Time last_camera_stamp_;
    ros::Time last_lidar_stamp_;
    ros::Time last_camera_arrival_;
    ros::Time last_lidar_arrival_;

    int camera_count_ = 0;
    int lidar_count_ = 0;

    std::deque<TimestampPair> delay_queue_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_diagnostics_node");

    try
    {
        SyncDiagnostics diagnostics;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("Sync Diagnostics Node failed: %s", e.what());
        return 1;
    }

    return 0;
}
