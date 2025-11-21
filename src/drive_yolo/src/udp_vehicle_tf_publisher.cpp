/**
 * UDP Vehicle TF Publisher Node
 *
 * Receives vehicle position data via UDP socket:
 * - Format: roll, pitch, yaw, x, y (5 doubles)
 * - Publishes TF transform from map/world to vehicle base_link
 *
 * Usage:
 *   rosrun drive_yolo udp_vehicle_tf_publisher _port:=5000
 */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>

struct VehiclePose {
    double roll, pitch, yaw;
    double x, y;
    ros::Time timestamp;
};

class UDPVehicleTFPublisher {
public:
    UDPVehicleTFPublisher() : nh_(""), pnh_("~"), running_(true) {
        // Parameters
        pnh_.param<int>("port", udp_port_, 5000);
        pnh_.param<std::string>("world_frame", world_frame_, "map");
        pnh_.param<std::string>("vehicle_frame", vehicle_frame_, "base_link");
        pnh_.param<double>("publish_rate", publish_rate_, 50.0);  // 50 Hz
        pnh_.param<bool>("publish_odom", publish_odom_, true);
        pnh_.param<double>("z_offset", z_offset_, 0.0);  // Height above ground

        // Publishers
        if (publish_odom_) {
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/vehicle/odom", 10);
            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vehicle/pose", 10);
        }

        ROS_INFO("=== UDP Vehicle TF Publisher ===");
        ROS_INFO("Listening on UDP port: %d", udp_port_);
        ROS_INFO("Publishing TF: %s -> %s", world_frame_.c_str(), vehicle_frame_.c_str());
        ROS_INFO("Publish rate: %.1f Hz", publish_rate_);
        ROS_INFO("Z offset: %.2f m", z_offset_);
        ROS_INFO("===============================");

        // Start UDP receiver thread
        udp_thread_ = std::thread(&UDPVehicleTFPublisher::udpReceiverThread, this);

        // Start TF publisher timer
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                        &UDPVehicleTFPublisher::publishTF, this);
    }

    ~UDPVehicleTFPublisher() {
        running_ = false;
        if (udp_thread_.joinable()) {
            udp_thread_.join();
        }
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher odom_pub_, pose_pub_;
    ros::Timer publish_timer_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::thread udp_thread_;
    std::atomic<bool> running_;
    int sockfd_ = -1;

    int udp_port_;
    std::string world_frame_, vehicle_frame_;
    double publish_rate_;
    bool publish_odom_;
    double z_offset_;

    VehiclePose current_pose_;
    std::mutex pose_mutex_;

    bool has_received_data_ = false;
    size_t packets_received_ = 0;

    void udpReceiverThread() {
        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            ROS_ERROR("Failed to create UDP socket");
            return;
        }

        // Set socket to non-blocking with timeout
        struct timeval tv;
        tv.tv_sec = 1;  // 1 second timeout
        tv.tv_usec = 0;
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Bind socket
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(udp_port_);

        if (bind(sockfd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            ROS_ERROR("Failed to bind UDP socket to port %d", udp_port_);
            close(sockfd_);
            sockfd_ = -1;
            return;
        }

        ROS_INFO("UDP socket bound successfully, waiting for data...");

        // Receive loop
        while (running_ && ros::ok()) {
            // Expect 5 doubles: roll, pitch, yaw, x, y
            double buffer[5];
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);

            ssize_t recv_len = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                                       (struct sockaddr*)&client_addr, &client_len);

            if (recv_len == sizeof(buffer)) {
                std::lock_guard<std::mutex> lock(pose_mutex_);

                current_pose_.roll = buffer[0];
                current_pose_.pitch = buffer[1];
                current_pose_.yaw = buffer[2];
                current_pose_.x = buffer[3];
                current_pose_.y = buffer[4];
                current_pose_.timestamp = ros::Time::now();

                if (!has_received_data_) {
                    ROS_INFO("First UDP packet received from %s:%d",
                            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                    has_received_data_ = true;
                }

                packets_received_++;

                ROS_DEBUG("Received pose: x=%.2f, y=%.2f, yaw=%.2f°",
                         current_pose_.x, current_pose_.y, current_pose_.yaw * 180.0 / M_PI);

            } else if (recv_len > 0) {
                ROS_WARN_THROTTLE(5.0, "Received invalid packet size: %zd bytes (expected %zu)",
                                 recv_len, sizeof(buffer));
            }
            // Timeout or error - continue loop
        }

        ROS_INFO("UDP receiver thread shutting down");
    }

    void publishTF(const ros::TimerEvent&) {
        if (!has_received_data_) {
            ROS_WARN_THROTTLE(5.0, "No UDP data received yet (packets: %zu)", packets_received_);
            return;
        }

        VehiclePose pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose = current_pose_;
        }

        // Create transform
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = world_frame_;
        transform.child_frame_id = vehicle_frame_;

        transform.transform.translation.x = pose.x;
        transform.transform.translation.y = pose.y;
        transform.transform.translation.z = z_offset_;

        // Convert RPY to quaternion
        tf2::Quaternion q;
        q.setRPY(pose.roll, pose.pitch, pose.yaw);

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_.sendTransform(transform);

        // Publish odometry if enabled
        if (publish_odom_) {
            nav_msgs::Odometry odom_msg;
            odom_msg.header = transform.header;
            odom_msg.child_frame_id = vehicle_frame_;

            odom_msg.pose.pose.position.x = pose.x;
            odom_msg.pose.pose.position.y = pose.y;
            odom_msg.pose.pose.position.z = z_offset_;

            odom_msg.pose.pose.orientation = transform.transform.rotation;

            odom_pub_.publish(odom_msg);

            // Also publish PoseStamped
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = transform.header;
            pose_msg.pose = odom_msg.pose.pose;
            pose_pub_.publish(pose_msg);
        }

        double age = (ros::Time::now() - pose.timestamp).toSec();
        if (age > 0.5) {
            ROS_WARN_THROTTLE(2.0, "Stale pose data (%.2f seconds old)", age);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_vehicle_tf_publisher");
    UDPVehicleTFPublisher node;
    ros::spin();
    return 0;
}
