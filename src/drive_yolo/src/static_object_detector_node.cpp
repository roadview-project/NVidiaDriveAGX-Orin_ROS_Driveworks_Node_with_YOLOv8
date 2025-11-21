/**
 * Static Object Detector Node
 *
 * Detects static objects at known world positions using LiDAR data.
 * Checks if points exist within a bounding box at given X,Y world coordinates.
 *
 * Usage:
 *   - Configure static object positions via ROS parameters
 *   - Publishes bounding boxes when LiDAR points are detected inside
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "drive_yolo/Detection.h"
#include "drive_yolo/Detections.h"

#include <vector>
#include <cmath>
#include <string>

struct StaticObject {
    int id;
    double x, y, z;          // Position in world frame
    double width, height, depth;  // Bounding box size
    std::string label;
    bool detected;
    int detection_count;
    ros::Time last_detection;
};

class StaticObjectDetectorNode {
public:
    StaticObjectDetectorNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_) {
        // Parameters
        pnh_.param<std::string>("world_frame", world_frame_, "map");
        pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
        pnh_.param<int>("min_points_threshold", min_points_threshold_, 5);
        pnh_.param<double>("detection_timeout", detection_timeout_, 2.0);

        // Load static object definitions
        loadStaticObjects();

        // Subscribers and publishers
        cloud_sub_ = nh_.subscribe("/lidar/points", 10,
                                   &StaticObjectDetectorNode::cloudCallback, this);

        detection_pub_ = nh_.advertise<drive_yolo::Detections>("/static_objects/detections", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/static_objects/markers", 1);

        // Timer for periodic marker publishing
        marker_timer_ = nh_.createTimer(ros::Duration(0.5),
                                       &StaticObjectDetectorNode::publishMarkers, this);

        ROS_INFO("=== Static Object Detector Started ===");
        ROS_INFO("World frame: %s", world_frame_.c_str());
        ROS_INFO("LiDAR frame: %s", lidar_frame_.c_str());
        ROS_INFO("Min points threshold: %d", min_points_threshold_);
        ROS_INFO("Loaded %zu static objects", static_objects_.size());
        ROS_INFO("=====================================");

        ros::Duration(1.0).sleep();  // Wait for TF
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher detection_pub_;
    ros::Publisher marker_pub_;
    ros::Timer marker_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string world_frame_, lidar_frame_;
    int min_points_threshold_;
    double detection_timeout_;

    std::vector<StaticObject> static_objects_;

    void loadStaticObjects() {
        // Load static objects from ROS parameters
        // Format: static_objects/object_0/x, y, z, width, height, depth, label

        XmlRpc::XmlRpcValue objects_config;
        if (!pnh_.getParam("static_objects", objects_config)) {
            ROS_WARN("No static_objects parameter found, using default test objects");

            // Create some test objects
            static_objects_.push_back({0, 10.0, 0.0, 0.0, 2.0, 2.0, 1.0, "pole_1", false, 0, ros::Time()});
            static_objects_.push_back({1, 20.0, 5.0, 0.0, 3.0, 2.5, 2.0, "building_1", false, 0, ros::Time()});
            return;
        }

        if (objects_config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("static_objects parameter must be an array");
            return;
        }

        for (int i = 0; i < objects_config.size(); ++i) {
            XmlRpc::XmlRpcValue& obj = objects_config[i];

            if (obj.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN("Object %d is not a struct, skipping", i);
                continue;
            }

            StaticObject static_obj;
            static_obj.id = i;

            if (obj.hasMember("x")) static_obj.x = static_cast<double>(obj["x"]);
            if (obj.hasMember("y")) static_obj.y = static_cast<double>(obj["y"]);
            if (obj.hasMember("z")) static_obj.z = static_cast<double>(obj["z"]);
            else static_obj.z = 0.0;

            if (obj.hasMember("width")) static_obj.width = static_cast<double>(obj["width"]);
            else static_obj.width = 1.0;

            if (obj.hasMember("height")) static_obj.height = static_cast<double>(obj["height"]);
            else static_obj.height = 2.0;

            if (obj.hasMember("depth")) static_obj.depth = static_cast<double>(obj["depth"]);
            else static_obj.depth = 1.0;

            if (obj.hasMember("label")) static_obj.label = static_cast<std::string>(obj["label"]);
            else static_obj.label = "object_" + std::to_string(i);

            static_obj.detected = false;
            static_obj.detection_count = 0;

            static_objects_.push_back(static_obj);

            ROS_INFO("Loaded object %d: '%s' at (%.1f, %.1f, %.1f), size: %.1fx%.1fx%.1f",
                    static_obj.id, static_obj.label.c_str(),
                    static_obj.x, static_obj.y, static_obj.z,
                    static_obj.width, static_obj.height, static_obj.depth);
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // Transform point cloud to world frame
        sensor_msgs::PointCloud2 cloud_world;

        try {
            if (!tf_buffer_.canTransform(world_frame_, cloud_msg->header.frame_id,
                                        cloud_msg->header.stamp, ros::Duration(0.1))) {
                ROS_WARN_THROTTLE(5.0, "Transform not available: %s -> %s",
                                 cloud_msg->header.frame_id.c_str(), world_frame_.c_str());
                return;
            }

            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                world_frame_, cloud_msg->header.frame_id,
                cloud_msg->header.stamp, ros::Duration(0.1));

            // Transform all points to world frame
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

            // Check each static object
            for (auto& obj : static_objects_) {
                int points_in_bbox = 0;

                // Reset iterator
                iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*cloud_msg, "x");
                iter_y = sensor_msgs::PointCloud2ConstIterator<float>(*cloud_msg, "y");
                iter_z = sensor_msgs::PointCloud2ConstIterator<float>(*cloud_msg, "z");

                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                        continue;
                    }

                    // Transform point to world frame
                    geometry_msgs::PointStamped pt_lidar, pt_world;
                    pt_lidar.header = cloud_msg->header;
                    pt_lidar.point.x = *iter_x;
                    pt_lidar.point.y = *iter_y;
                    pt_lidar.point.z = *iter_z;

                    tf2::doTransform(pt_lidar, pt_world, transform);

                    // Check if point is inside bounding box
                    double dx = std::abs(pt_world.point.x - obj.x);
                    double dy = std::abs(pt_world.point.y - obj.y);
                    double dz = std::abs(pt_world.point.z - obj.z);

                    if (dx <= obj.width / 2.0 &&
                        dy <= obj.depth / 2.0 &&
                        dz <= obj.height / 2.0) {
                        points_in_bbox++;
                    }
                }

                // Update detection status
                bool was_detected = obj.detected;
                obj.detected = (points_in_bbox >= min_points_threshold_);

                if (obj.detected) {
                    obj.detection_count++;
                    obj.last_detection = ros::Time::now();

                    if (!was_detected) {
                        ROS_INFO("Object '%s' (ID %d) detected! Points: %d",
                                obj.label.c_str(), obj.id, points_in_bbox);
                    }
                } else if (was_detected) {
                    ROS_INFO("Object '%s' (ID %d) lost", obj.label.c_str(), obj.id);
                }

                ROS_DEBUG("Object '%s': %d points %s",
                         obj.label.c_str(), points_in_bbox,
                         obj.detected ? "DETECTED" : "not detected");
            }

            // Publish detections
            publishDetections(cloud_msg->header);

        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "Transform exception: %s", ex.what());
            return;
        }
    }

    void publishDetections(const std_msgs::Header& header) {
        drive_yolo::Detections detections_msg;
        detections_msg.header = header;
        detections_msg.header.frame_id = world_frame_;

        for (const auto& obj : static_objects_) {
            if (obj.detected) {
                drive_yolo::Detection det;
                det.class_id = 99;  // Special ID for static objects
                det.class_name = obj.label;
                det.confidence = std::min(1.0f, obj.detection_count / 10.0f);

                // Store 3D position in unused fields (or create new message type)
                det.x = static_cast<float>(obj.x);
                det.y = static_cast<float>(obj.y);
                det.width = static_cast<float>(obj.width);

                detections_msg.detections.push_back(det);
            }
        }

        if (!detections_msg.detections.empty()) {
            detection_pub_.publish(detections_msg);
        }
    }

    void publishMarkers(const ros::TimerEvent&) {
        visualization_msgs::MarkerArray marker_array;

        for (const auto& obj : static_objects_) {
            // Bounding box marker
            visualization_msgs::Marker box;
            box.header.frame_id = world_frame_;
            box.header.stamp = ros::Time::now();
            box.ns = "static_objects";
            box.id = obj.id;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;

            box.pose.position.x = obj.x;
            box.pose.position.y = obj.y;
            box.pose.position.z = obj.z;
            box.pose.orientation.w = 1.0;

            box.scale.x = obj.width;
            box.scale.y = obj.depth;
            box.scale.z = obj.height;

            if (obj.detected) {
                // Green when detected
                box.color.r = 0.0;
                box.color.g = 1.0;
                box.color.b = 0.0;
                box.color.a = 0.5;
            } else {
                // Blue when not detected
                box.color.r = 0.0;
                box.color.g = 0.5;
                box.color.b = 1.0;
                box.color.a = 0.3;
            }

            marker_array.markers.push_back(box);

            // Text label
            visualization_msgs::Marker text;
            text.header = box.header;
            text.ns = "static_objects_labels";
            text.id = obj.id + 1000;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;

            text.pose.position.x = obj.x;
            text.pose.position.y = obj.y;
            text.pose.position.z = obj.z + obj.height / 2.0 + 0.5;

            text.scale.z = 0.5;

            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;

            std::stringstream ss;
            ss << obj.label << "\n";
            if (obj.detected) {
                ss << "DETECTED (" << obj.detection_count << ")";
            } else {
                double time_since = (ros::Time::now() - obj.last_detection).toSec();
                if (time_since < detection_timeout_) {
                    ss << "Recent (" << std::fixed << std::setprecision(1) << time_since << "s ago)";
                }
            }
            text.text = ss.str();

            marker_array.markers.push_back(text);
        }

        marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_object_detector_node");
    StaticObjectDetectorNode node;
    ros::spin();
    return 0;
}
