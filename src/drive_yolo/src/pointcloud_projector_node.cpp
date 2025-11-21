/**
 * Point Cloud Projector Node
 *
 * Projects 3D LiDAR points onto 2D camera image plane using:
 * - Camera calibration (intrinsics + distortion)
 * - TF transforms (LiDAR -> Camera)
 *
 * Publishes projected points as a custom message for debugging.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// Custom message for projected points (will be defined in msg/)
#include "drive_yolo/ProjectedPoints.h"
#include "drive_yolo/ProjectedPoint.h"

class PointCloudProjectorNode {
public:
    PointCloudProjectorNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_) {
        // Frame parameters
        pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
        pnh_.param<std::string>("camera_frame", camera_frame_, "camera");
        pnh_.param<double>("transform_timeout", transform_timeout_, 0.1);

        // Camera calibration
        std::vector<double> camera_matrix_data, dist_coeffs_data;
        pnh_.getParam("camera_matrix", camera_matrix_data);
        pnh_.getParam("distortion_coefficients", dist_coeffs_data);

        if (camera_matrix_data.size() == 9) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
            ROS_INFO("Camera matrix loaded");
        } else {
            ROS_ERROR("Invalid camera_matrix parameter");
        }

        if (dist_coeffs_data.size() >= 4) {
            dist_coeffs_ = cv::Mat(1, dist_coeffs_data.size(), CV_64F, dist_coeffs_data.data()).clone();
            ROS_INFO("Distortion coefficients loaded");
        }

        pnh_.param<int>("image_width", image_width_, 1920);
        pnh_.param<int>("image_height", image_height_, 1080);
        pnh_.param<bool>("debug_visualization", debug_viz_, true);

        // Subscribers and publishers
        cloud_sub_ = nh_.subscribe("/lidar/points_filtered", 10,
                                   &PointCloudProjectorNode::cloudCallback, this);
        projected_pub_ = nh_.advertise<drive_yolo::ProjectedPoints>("/lidar/points_projected", 10);

        if (debug_viz_) {
            debug_image_pub_ = nh_.advertise<sensor_msgs::Image>("/pointcloud_projector/debug_image", 1);
            ROS_INFO("Debug visualization: ENABLED");
        }

        ROS_INFO("=== Point Cloud Projector Node Started ===");
        ROS_INFO("LiDAR frame: %s", lidar_frame_.c_str());
        ROS_INFO("Camera frame: %s", camera_frame_.c_str());
        ROS_INFO("Image size: %dx%d", image_width_, image_height_);
        ROS_INFO("=========================================");

        // Wait for TF
        ros::Duration(1.0).sleep();
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher projected_pub_;
    ros::Publisher debug_image_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    int image_width_, image_height_;
    bool debug_viz_;

    std::string lidar_frame_, camera_frame_;
    double transform_timeout_;

    size_t total_points_processed_ = 0;
    size_t total_points_projected_ = 0;

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // Get transform
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(camera_frame_, lidar_frame_,
                                                  cloud_msg->header.stamp,
                                                  ros::Duration(transform_timeout_));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "Transform failed: %s", ex.what());
            return;
        }

        // Extract rotation and translation
        cv::Mat R, t;
        tf2::Quaternion q(transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z,
                         transform.transform.rotation.w);
        tf2::Matrix3x3 tf2_rot(q);

        R = (cv::Mat_<double>(3, 3) <<
            tf2_rot[0][0], tf2_rot[0][1], tf2_rot[0][2],
            tf2_rot[1][0], tf2_rot[1][1], tf2_rot[1][2],
            tf2_rot[2][0], tf2_rot[2][1], tf2_rot[2][2]);

        t = (cv::Mat_<double>(3, 1) <<
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        // Parse point cloud
        std::vector<cv::Point3f> points_3d;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
                points_3d.emplace_back(*iter_x, *iter_y, *iter_z);
            }
        }

        total_points_processed_ = points_3d.size();

        if (points_3d.empty()) {
            ROS_DEBUG("No valid points to project");
            return;
        }

        // Project points
        std::vector<cv::Point2f> points_2d;
        cv::projectPoints(points_3d, R, t, camera_matrix_, dist_coeffs_, points_2d);

        // Filter valid projections
        drive_yolo::ProjectedPoints projected_msg;
        projected_msg.header = cloud_msg->header;
        projected_msg.header.frame_id = camera_frame_;

        cv::Mat debug_image;
        if (debug_viz_) {
            debug_image = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
        }

        size_t valid_count = 0;
        for (size_t i = 0; i < points_2d.size(); ++i) {
            const auto& pt_2d = points_2d[i];
            const auto& pt_3d = points_3d[i];

            // Check if projection is within image bounds
            if (pt_2d.x >= 0 && pt_2d.x < image_width_ &&
                pt_2d.y >= 0 && pt_2d.y < image_height_ &&
                pt_3d.z > 0) {  // In front of camera

                drive_yolo::ProjectedPoint proj_pt;
                proj_pt.point_3d.x = pt_3d.x;
                proj_pt.point_3d.y = pt_3d.y;
                proj_pt.point_3d.z = pt_3d.z;
                proj_pt.pixel_x = pt_2d.x;
                proj_pt.pixel_y = pt_2d.y;
                proj_pt.distance = std::sqrt(pt_3d.x*pt_3d.x + pt_3d.y*pt_3d.y + pt_3d.z*pt_3d.z);

                projected_msg.points.push_back(proj_pt);
                valid_count++;

                if (debug_viz_) {
                    // Color by distance
                    double dist = proj_pt.distance;
                    cv::Vec3b color;
                    if (dist < 10.0) color = cv::Vec3b(0, 255, 0);        // Green - close
                    else if (dist < 30.0) color = cv::Vec3b(0, 255, 255); // Yellow - medium
                    else color = cv::Vec3b(0, 0, 255);                    // Red - far

                    cv::circle(debug_image, cv::Point(pt_2d.x, pt_2d.y), 2, color, -1);
                }
            }
        }

        total_points_projected_ = valid_count;

        projected_pub_.publish(projected_msg);

        if (debug_viz_ && !debug_image.empty()) {
            // Add stats text
            std::stringstream ss;
            ss << "Points: " << valid_count << "/" << total_points_processed_;
            cv::putText(debug_image, ss.str(), cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(
                cloud_msg->header, "bgr8", debug_image).toImageMsg();
            debug_image_pub_.publish(img_msg);
        }

        ROS_DEBUG("Projected %zu/%zu points (%.1f%%)",
                 valid_count, total_points_processed_,
                 100.0 * valid_count / total_points_processed_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_projector_node");
    PointCloudProjectorNode node;
    ros::spin();
    return 0;
}
