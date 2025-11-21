/**
 * Point Cloud Filter Node
 *
 * Applies various filters to incoming point clouds:
 * - Distance filtering (min/max range)
 * - ROI filtering (region of interest)
 * - Ground plane removal
 * - Outlier removal
 *
 * For easier debugging with visualization and configurable parameters.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>
#include <cmath>

class PointCloudFilterNode {
public:
    PointCloudFilterNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_) {
        // Distance filtering
        pnh_.param<double>("min_distance", min_distance_, 0.5);
        pnh_.param<double>("max_distance", max_distance_, 100.0);

        // ROI filtering
        pnh_.param<bool>("enable_roi_filtering", enable_roi_filtering_, true);
        pnh_.param<double>("roi_x_min", roi_x_min_, 0.0);
        pnh_.param<double>("roi_x_max", roi_x_max_, 50.0);
        pnh_.param<double>("roi_y_min", roi_y_min_, -10.0);
        pnh_.param<double>("roi_y_max", roi_y_max_, 10.0);
        pnh_.param<double>("roi_z_min", roi_z_min_, -2.0);
        pnh_.param<double>("roi_z_max", roi_z_max_, 10.0);

        // Ground filtering
        pnh_.param<bool>("enable_ground_filtering", enable_ground_filtering_, false);
        pnh_.param<double>("ground_z_threshold", ground_z_threshold_, -1.0);

        // Visualization
        pnh_.param<bool>("enable_visualization", enable_visualization_, true);
        pnh_.param<std::string>("target_frame", target_frame_, "base_link");

        // Topics
        std::string input_topic, output_topic;
        pnh_.param<std::string>("input_topic", input_topic, "/lidar/points");
        pnh_.param<std::string>("output_topic", output_topic, "/lidar/points_filtered");

        // Publishers and subscribers
        cloud_sub_ = nh_.subscribe(input_topic, 10, &PointCloudFilterNode::cloudCallback, this);
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 10);

        if (enable_visualization_) {
            roi_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/pointcloud_filter/roi_markers", 1);
            stats_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pointcloud_filter/stats", 1);
            publishROIMarkers();
        }

        ROS_INFO("=== Point Cloud Filter Node Started ===");
        ROS_INFO("Input topic: %s", input_topic.c_str());
        ROS_INFO("Output topic: %s", output_topic.c_str());
        ROS_INFO("Distance filter: [%.1f, %.1f] m", min_distance_, max_distance_);
        ROS_INFO("ROI filtering: %s", enable_roi_filtering_ ? "ENABLED" : "DISABLED");
        if (enable_roi_filtering_) {
            ROS_INFO("  X: [%.1f, %.1f] m", roi_x_min_, roi_x_max_);
            ROS_INFO("  Y: [%.1f, %.1f] m", roi_y_min_, roi_y_max_);
            ROS_INFO("  Z: [%.1f, %.1f] m", roi_z_min_, roi_z_max_);
        }
        ROS_INFO("Ground filtering: %s (Z < %.2f m)",
                 enable_ground_filtering_ ? "ENABLED" : "DISABLED", ground_z_threshold_);
        ROS_INFO("======================================");

        stats_timer_ = nh_.createTimer(ros::Duration(1.0), &PointCloudFilterNode::publishStats, this);
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher roi_marker_pub_;
    ros::Publisher stats_marker_pub_;
    ros::Timer stats_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    double min_distance_, max_distance_;
    bool enable_roi_filtering_;
    double roi_x_min_, roi_x_max_;
    double roi_y_min_, roi_y_max_;
    double roi_z_min_, roi_z_max_;
    bool enable_ground_filtering_;
    double ground_z_threshold_;
    bool enable_visualization_;
    std::string target_frame_;

    // Statistics
    size_t total_points_in_ = 0;
    size_t total_points_out_ = 0;
    size_t points_filtered_distance_ = 0;
    size_t points_filtered_roi_ = 0;
    size_t points_filtered_ground_ = 0;
    double last_processing_time_ms_ = 0.0;

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        auto start_time = ros::Time::now();

        sensor_msgs::PointCloud2 filtered_cloud;
        filtered_cloud.header = cloud_msg->header;
        filtered_cloud.height = 1;
        filtered_cloud.is_dense = false;
        filtered_cloud.is_bigendian = cloud_msg->is_bigendian;

        // Setup fields (x, y, z, intensity if available)
        filtered_cloud.fields = cloud_msg->fields;
        filtered_cloud.point_step = cloud_msg->point_step;

        // Temporary storage for filtered points
        std::vector<uint8_t> filtered_data;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

        size_t input_count = 0;
        size_t output_count = 0;
        size_t dist_filtered = 0;
        size_t roi_filtered = 0;
        size_t ground_filtered = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            input_count++;

            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // Skip invalid points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            // Distance filter
            double distance = std::sqrt(x*x + y*y + z*z);
            if (distance < min_distance_ || distance > max_distance_) {
                dist_filtered++;
                continue;
            }

            // ROI filter
            if (enable_roi_filtering_) {
                if (x < roi_x_min_ || x > roi_x_max_ ||
                    y < roi_y_min_ || y > roi_y_max_ ||
                    z < roi_z_min_ || z > roi_z_max_) {
                    roi_filtered++;
                    continue;
                }
            }

            // Ground filter
            if (enable_ground_filtering_ && z < ground_z_threshold_) {
                ground_filtered++;
                continue;
            }

            // Point passed all filters - copy raw bytes
	    const uint8_t* point_data = reinterpret_cast<const uint8_t*>(&(*iter_x)) - 
                            cloud_msg->fields[0].offset;
            filtered_data.insert(filtered_data.end(),
                               point_data,
                               point_data + cloud_msg->point_step);
            output_count++;
        }

        // Update global statistics
        total_points_in_ = input_count;
        total_points_out_ = output_count;
        points_filtered_distance_ = dist_filtered;
        points_filtered_roi_ = roi_filtered;
        points_filtered_ground_ = ground_filtered;

        // Set filtered cloud data
        filtered_cloud.data = filtered_data;
        filtered_cloud.width = output_count;
        filtered_cloud.row_step = filtered_cloud.point_step * filtered_cloud.width;

        filtered_cloud_pub_.publish(filtered_cloud);

        last_processing_time_ms_ = (ros::Time::now() - start_time).toSec() * 1000.0;

        ROS_DEBUG("Filtered: %zu -> %zu points (%.1f%%), time: %.2f ms",
                 input_count, output_count,
                 input_count > 0 ? 100.0 * output_count / input_count : 0.0,
                 last_processing_time_ms_);
    }

    void publishROIMarkers() {
        if (!enable_roi_filtering_) return;

        visualization_msgs::MarkerArray marker_array;

        // Create ROI box marker
        visualization_msgs::Marker roi_box;
        roi_box.header.frame_id = target_frame_;
        roi_box.header.stamp = ros::Time::now();
        roi_box.ns = "roi_box";
        roi_box.id = 0;
        roi_box.type = visualization_msgs::Marker::CUBE;
        roi_box.action = visualization_msgs::Marker::ADD;

        roi_box.pose.position.x = (roi_x_min_ + roi_x_max_) / 2.0;
        roi_box.pose.position.y = (roi_y_min_ + roi_y_max_) / 2.0;
        roi_box.pose.position.z = (roi_z_min_ + roi_z_max_) / 2.0;
        roi_box.pose.orientation.w = 1.0;

        roi_box.scale.x = roi_x_max_ - roi_x_min_;
        roi_box.scale.y = roi_y_max_ - roi_y_min_;
        roi_box.scale.z = roi_z_max_ - roi_z_min_;

        roi_box.color.r = 0.0;
        roi_box.color.g = 1.0;
        roi_box.color.b = 0.0;
        roi_box.color.a = 0.2;

        marker_array.markers.push_back(roi_box);
        roi_marker_pub_.publish(marker_array);
    }

    void publishStats(const ros::TimerEvent&) {
        if (!enable_visualization_) return;

        // Create text marker with statistics
        visualization_msgs::Marker stats_marker;
        stats_marker.header.frame_id = target_frame_;
        stats_marker.header.stamp = ros::Time::now();
        stats_marker.ns = "filter_stats";
        stats_marker.id = 0;
        stats_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        stats_marker.action = visualization_msgs::Marker::ADD;

        stats_marker.pose.position.x = roi_x_max_ + 5.0;
        stats_marker.pose.position.y = 0.0;
        stats_marker.pose.position.z = 5.0;
        stats_marker.pose.orientation.w = 1.0;

        stats_marker.scale.z = 1.0;  // Text height

        stats_marker.color.r = 1.0;
        stats_marker.color.g = 1.0;
        stats_marker.color.b = 1.0;
        stats_marker.color.a = 1.0;

        std::stringstream ss;
        ss << "PointCloud Filter Stats\n";
        ss << "In: " << total_points_in_ << " points\n";
        ss << "Out: " << total_points_out_ << " points\n";
        if (total_points_in_ > 0) {
            ss << "Pass rate: " << std::fixed << std::setprecision(1)
               << (100.0 * total_points_out_ / total_points_in_) << "%\n";
        }
        ss << "Filtered by distance: " << points_filtered_distance_ << "\n";
        ss << "Filtered by ROI: " << points_filtered_roi_ << "\n";
        ss << "Filtered by ground: " << points_filtered_ground_ << "\n";
        ss << "Processing time: " << std::fixed << std::setprecision(2)
           << last_processing_time_ms_ << " ms";

        stats_marker.text = ss.str();

        stats_marker_pub_.publish(stats_marker);

        // Also republish ROI markers
        publishROIMarkers();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_filter_node");
    PointCloudFilterNode node;
    ros::spin();
    return 0;
}
