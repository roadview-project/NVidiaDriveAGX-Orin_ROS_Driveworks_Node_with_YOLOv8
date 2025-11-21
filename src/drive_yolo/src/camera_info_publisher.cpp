/**
 * Camera Info Publisher Node
 *
 * This node publishes camera calibration information (CameraInfo) for the
 * Entron camera, enabling proper pointcloud overlay visualization in RViz.
 *
 * The calibration parameters are loaded from the fusion lidar configuration
 * and published at a fixed rate to support real-time visualization.
 */

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>

class CameraInfoPublisher
{
public:
    CameraInfoPublisher() : nh_("~")
    {
        // Load parameters
        nh_.param<std::string>("camera_frame", camera_frame_, "camera_entron");
        nh_.param<int>("image_width", image_width_, 3848);
        nh_.param<int>("image_height", image_height_, 2168);
        nh_.param<double>("publish_rate", publish_rate_, 30.0);

        // Load camera matrix (3x3)
        std::vector<double> camera_matrix;
        if (!nh_.getParam("camera_matrix", camera_matrix) || camera_matrix.size() != 9)
        {
            ROS_ERROR("Camera matrix must be a 9-element array (3x3 matrix in row-major order)");
            camera_matrix = {3145.24132, 0.0, 1887.68485,
                           0.0, 3166.08502, 990.62304,
                           0.0, 0.0, 1.0};
            ROS_WARN("Using default camera matrix from lidar_camera_fusion.launch");
        }

        // Load distortion coefficients
        std::vector<double> distortion_coeffs;
        if (!nh_.getParam("distortion_coefficients", distortion_coeffs) || distortion_coeffs.size() != 5)
        {
            ROS_ERROR("Distortion coefficients must be a 5-element array [k1, k2, p1, p2, k3]");
            distortion_coeffs = {-0.298671, 0.087807, 0.000383, 0.000551, 0.000000};
            ROS_WARN("Using default distortion coefficients from lidar_camera_fusion.launch");
        }

        // Initialize CameraInfo message
        initializeCameraInfo(camera_matrix, distortion_coeffs);

        // Create publisher
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 10);

        // Create timer for periodic publishing
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &CameraInfoPublisher::publishCameraInfo, this);

        ROS_INFO("Camera Info Publisher initialized");
        ROS_INFO("  Camera frame: %s", camera_frame_.c_str());
        ROS_INFO("  Image size: %dx%d", image_width_, image_height_);
        ROS_INFO("  Publishing rate: %.1f Hz", publish_rate_);
        ROS_INFO("  Topic: /camera/camera_info");
    }

private:
    void initializeCameraInfo(const std::vector<double>& camera_matrix,
                             const std::vector<double>& distortion_coeffs)
    {
        camera_info_.header.frame_id = camera_frame_;
        camera_info_.width = image_width_;
        camera_info_.height = image_height_;

        // Set distortion model (plumb_bob is standard for OpenCV calibration)
        camera_info_.distortion_model = "plumb_bob";

        // Set distortion coefficients [k1, k2, p1, p2, k3]
        camera_info_.D.resize(5);
        for (int i = 0; i < 5; i++)
        {
            camera_info_.D[i] = distortion_coeffs[i];
        }

        // Set camera matrix K (3x3)
        // K = [fx  0  cx]
        //     [ 0 fy  cy]
        //     [ 0  0   1]
        for (int i = 0; i < 9; i++)
        {
            camera_info_.K[i] = camera_matrix[i];
        }

        // Set rectification matrix R (3x3) - identity for unrectified image
        camera_info_.R[0] = 1.0; camera_info_.R[1] = 0.0; camera_info_.R[2] = 0.0;
        camera_info_.R[3] = 0.0; camera_info_.R[4] = 1.0; camera_info_.R[5] = 0.0;
        camera_info_.R[6] = 0.0; camera_info_.R[7] = 0.0; camera_info_.R[8] = 1.0;

        // Set projection matrix P (3x4)
        // P = [fx'  0  cx' Tx]
        //     [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        // For monocular camera: Tx = Ty = 0, fx' = fx, fy' = fy, cx' = cx, cy' = cy
        camera_info_.P[0]  = camera_matrix[0]; // fx
        camera_info_.P[1]  = 0.0;
        camera_info_.P[2]  = camera_matrix[2]; // cx
        camera_info_.P[3]  = 0.0;              // Tx
        camera_info_.P[4]  = 0.0;
        camera_info_.P[5]  = camera_matrix[4]; // fy
        camera_info_.P[6]  = camera_matrix[5]; // cy
        camera_info_.P[7]  = 0.0;              // Ty
        camera_info_.P[8]  = 0.0;
        camera_info_.P[9]  = 0.0;
        camera_info_.P[10] = 1.0;
        camera_info_.P[11] = 0.0;

        // Binning and ROI - not used
        camera_info_.binning_x = 0;
        camera_info_.binning_y = 0;
        camera_info_.roi.x_offset = 0;
        camera_info_.roi.y_offset = 0;
        camera_info_.roi.height = 0;
        camera_info_.roi.width = 0;
        camera_info_.roi.do_rectify = false;
    }

    void publishCameraInfo(const ros::TimerEvent&)
    {
        camera_info_.header.stamp = ros::Time::now();
        camera_info_pub_.publish(camera_info_);
    }

    ros::NodeHandle nh_;
    ros::Publisher camera_info_pub_;
    ros::Timer timer_;

    sensor_msgs::CameraInfo camera_info_;
    std::string camera_frame_;
    int image_width_;
    int image_height_;
    double publish_rate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_publisher");

    try
    {
        CameraInfoPublisher publisher;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("Camera Info Publisher failed: %s", e.what());
        return 1;
    }

    return 0;
}
