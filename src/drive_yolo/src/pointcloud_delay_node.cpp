/**
 * Pointcloud Delay Node
 *
 * This node adds artificial delay to pointcloud messages for testing
 * and visualization purposes. Useful for:
 * - Demonstrating synchronization issues
 * - Testing sync diagnostic and fusion configurations
 * - Creating before/after comparison videos
 *
 * Features:
 * - Configurable delay (milliseconds or frame count)
 * - Queue-based buffering
 * - Real-time statistics reporting
 * - Passthrough mode for quick enable/disable
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <deque>
#include <chrono>

class PointCloudDelayNode
{
public:
    PointCloudDelayNode() : nh_("~")
    {
        // Load parameters
        nh_.param<std::string>("input_topic", input_topic_, "/invz_summation_reflection");
        nh_.param<std::string>("output_topic", output_topic_, "/invz_summation_reflection_delayed");
        nh_.param<double>("delay_ms", delay_ms_, 50.0);  // 50ms default delay
        nh_.param<int>("frame_delay", frame_delay_, 0);  // 0 = use time delay, >0 = use frame delay
        nh_.param<int>("queue_size", queue_size_, 100);
        nh_.param<bool>("passthrough", passthrough_, false);
        nh_.param<double>("report_interval", report_interval_, 5.0);  // Report stats every 5 seconds
        nh_.param<bool>("preserve_timestamps", preserve_timestamps_, false);  // Keep original timestamps

        // Subscribers and publishers
        cloud_sub_ = nh_.subscribe(input_topic_, queue_size_, &PointCloudDelayNode::cloudCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        queue_size_pub_ = nh_.advertise<std_msgs::Int32>("/pointcloud_delay/queue_size", 10);

        // Timer for publishing delayed messages
        publish_timer_ = nh_.createTimer(ros::Duration(0.01), &PointCloudDelayNode::publishCallback, this);

        // Timer for statistics reporting
        report_timer_ = nh_.createTimer(ros::Duration(report_interval_),
                                        &PointCloudDelayNode::reportStatistics, this);

        ROS_INFO("=== POINTCLOUD DELAY NODE ===");
        ROS_INFO("Input topic:  %s", input_topic_.c_str());
        ROS_INFO("Output topic: %s", output_topic_.c_str());

        if (passthrough_) {
            ROS_WARN("PASSTHROUGH MODE: No delay applied");
        } else if (frame_delay_ > 0) {
            ROS_INFO("Frame delay: %d frames (%.1f ms at 30Hz)", frame_delay_, frame_delay_ * 33.33);
        } else {
            ROS_INFO("Time delay: %.1f ms", delay_ms_);
        }

        ROS_INFO("Queue size: %d", queue_size_);
        ROS_INFO("Preserve timestamps: %s", preserve_timestamps_ ? "YES" : "NO (shift by delay)");
        ROS_INFO("============================");

        start_time_ = ros::Time::now();
    }

private:
    struct DelayedMessage
    {
        sensor_msgs::PointCloud2::ConstPtr msg;
        ros::Time received_time;
        ros::Time publish_time;  // When this message should be published
    };

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        messages_received_++;

        // Passthrough mode - republish immediately
        if (passthrough_)
        {
            cloud_pub_.publish(msg);
            messages_published_++;
            return;
        }

        // Add to queue with calculated publish time
        DelayedMessage delayed_msg;
        delayed_msg.msg = msg;
        delayed_msg.received_time = ros::Time::now();

        if (frame_delay_ > 0)
        {
            // Frame-based delay: wait for N more frames
            delayed_msg.publish_time = delayed_msg.received_time;  // Will be published after N frames
        }
        else
        {
            // Time-based delay
            delayed_msg.publish_time = delayed_msg.received_time + ros::Duration(delay_ms_ / 1000.0);
        }

        message_queue_.push_back(delayed_msg);

        // Maintain queue size limit
        while (message_queue_.size() > static_cast<size_t>(queue_size_))
        {
            ROS_WARN_THROTTLE(1.0, "Queue full (%d), dropping oldest message", queue_size_);
            message_queue_.pop_front();
            messages_dropped_++;
        }

        // Publish queue size for monitoring
        std_msgs::Int32 queue_size_msg;
        queue_size_msg.data = message_queue_.size();
        queue_size_pub_.publish(queue_size_msg);
    }

    void publishCallback(const ros::TimerEvent&)
    {
        if (passthrough_ || message_queue_.empty())
        {
            return;
        }

        ros::Time now = ros::Time::now();

        // Frame-based delay: always publish from front if we have enough frames queued
        if (frame_delay_ > 0)
        {
            if (message_queue_.size() >= static_cast<size_t>(frame_delay_))
            {
                publishMessage(message_queue_.front());
                message_queue_.pop_front();
            }
        }
        // Time-based delay: publish all messages that are ready
        else
        {
            while (!message_queue_.empty() && message_queue_.front().publish_time <= now)
            {
                publishMessage(message_queue_.front());
                message_queue_.pop_front();
            }
        }
    }

    void publishMessage(const DelayedMessage& delayed_msg)
    {
        // Create a copy of the message to potentially modify timestamp
        sensor_msgs::PointCloud2 output_msg = *delayed_msg.msg;

        if (!preserve_timestamps_)
        {
            // Shift timestamp by the delay amount
            if (frame_delay_ > 0)
            {
                // Calculate actual delay based on frame count and average frame rate
                double avg_delay = frame_delay_ * 33.33 / 1000.0;  // Assuming ~30Hz
                output_msg.header.stamp = output_msg.header.stamp + ros::Duration(avg_delay);
            }
            else
            {
                output_msg.header.stamp = output_msg.header.stamp + ros::Duration(delay_ms_ / 1000.0);
            }
        }

        cloud_pub_.publish(output_msg);
        messages_published_++;

        // Track actual delay
        double actual_delay = (ros::Time::now() - delayed_msg.received_time).toSec() * 1000.0;
        total_delay_ += actual_delay;
    }

    void reportStatistics(const ros::TimerEvent&)
    {
        double uptime = (ros::Time::now() - start_time_).toSec();
        double avg_rate = messages_received_ / uptime;
        double avg_delay = messages_published_ > 0 ? total_delay_ / messages_published_ : 0.0;

        ROS_INFO("=== POINTCLOUD DELAY STATISTICS ===");
        ROS_INFO("Uptime: %.1f seconds", uptime);
        ROS_INFO("Messages: Received=%d, Published=%d, Dropped=%d",
                 messages_received_, messages_published_, messages_dropped_);
        ROS_INFO("Current queue size: %zu / %d", message_queue_.size(), queue_size_);
        ROS_INFO("Average rate: %.1f Hz", avg_rate);
        ROS_INFO("Average actual delay: %.1f ms", avg_delay);

        if (frame_delay_ > 0)
        {
            ROS_INFO("Frame delay: %d frames (queue needs %d to start publishing)",
                     frame_delay_, frame_delay_);
        }
        else
        {
            ROS_INFO("Configured delay: %.1f ms", delay_ms_);
        }

        if (message_queue_.size() >= static_cast<size_t>(queue_size_ * 0.9))
        {
            ROS_WARN("Queue nearly full! Consider increasing queue_size parameter.");
        }

        ROS_INFO("===================================");
    }

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher queue_size_pub_;
    ros::Timer publish_timer_;
    ros::Timer report_timer_;

    std::string input_topic_;
    std::string output_topic_;
    double delay_ms_;
    int frame_delay_;
    int queue_size_;
    bool passthrough_;
    double report_interval_;
    bool preserve_timestamps_;

    std::deque<DelayedMessage> message_queue_;

    // Statistics
    ros::Time start_time_;
    int messages_received_ = 0;
    int messages_published_ = 0;
    int messages_dropped_ = 0;
    double total_delay_ = 0.0;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_delay_node");

    try
    {
        PointCloudDelayNode delay_node;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("Pointcloud Delay Node failed: %s", e.what());
        return 1;
    }

    return 0;
}
