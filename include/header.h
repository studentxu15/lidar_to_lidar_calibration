#pragma
#ifndef LIDAR_CALIB_HEADER_H
#define LIDAR_CALIB_HEADER_H

#include <ros/ros.h>
#include <std_msgs/Header.h>          // std_msgs/Header
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// TF广播
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

// PCL基础及点云消息转换
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg, pcl::toROSMsg
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>

// STL
#include <deque>
#include <thread>
#include <mutex>
#include <string>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
typedef pcl::PointXYZI PointType; // Define the point type for PCL
typedef pcl::PointCloud<PointType> PointCloud; // Define the point cloud type
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr; // Define a pointer to the point cloud type

// 定义结构
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGRN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT ( VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)
using PointXYZIRT = VelodynePointXYZIRT;
typedef pcl::PointCloud<PointXYZIRT> PointXYZIRTCloud;
typedef pcl::PointCloud<PointXYZIRT>::Ptr PointXYZIRTCloudPtr;

class ParamServer
{
public:
    ros::NodeHandle nh; // Node handle for ROS
    ros::NodeHandle private_nh; // Private node handle for ROS

    string source_lidar_topic; // Source LiDAR topic
    string target_lidar_topic; // Target camera topic
    string initial_topic; // Initial transformation topic
    string flag_topic; // Header topic for initial pose
    string merged_cloud_topic; // Topic for merged point cloud
    string calibration_result_topic; // Topic for calibration result
    string marker_A_topic;
    string marker_B_topic;

    int source_lidar_queue_size; // Size of the source LiDAR queue
    double icp_score_threshold; // Threshold for ICP score

    double calib_x, calib_y, calib_z;
    double calib_roll, calib_pitch, calib_yaw;

    double box_A_lx, box_A_ly, box_A_lz, box_A_x, box_A_y, box_A_z, box_A_roll, box_A_pitch, box_A_yaw;
    double box_B_lx, box_B_ly, box_B_lz, box_B_x, box_B_y, box_B_z, box_B_roll, box_B_pitch, box_B_yaw;
    double board_l;

    ParamServer()
        : nh(), private_nh("~") // Initialize node handles
    {
        // Load parameters from the parameter server
        private_nh.param<string>("source_lidar_topic", source_lidar_topic, "/lidar_source/points");
        private_nh.param<string>("target_lidar_topic", target_lidar_topic, "/camera_target/image_raw");
        private_nh.param<string>("initial_topic", initial_topic, "/initialpose");
        private_nh.param<string>("flag_topic", flag_topic, "/header");
        private_nh.param<string>("merged_cloud_topic", merged_cloud_topic, "/merged_cloud");
        private_nh.param<string>("calibration_result_topic", calibration_result_topic, "/calibration_result");
        private_nh.param<string>("marker_A_topic", marker_A_topic, "/marker_A");
        private_nh.param<string>("marker_B_topic", marker_B_topic, "/marker_B");
        private_nh.param<int>("source_lidar_queue_size", source_lidar_queue_size, 10);
        private_nh.param<double>("icp_score_threshold", icp_score_threshold, 0.1);
        private_nh.param<double>("calib_x", calib_x, 0.0);
        private_nh.param<double>("calib_y", calib_y, 0.0);
        private_nh.param<double>("calib_z", calib_z, 0.0);
        private_nh.param<double>("calib_roll", calib_roll, 0.0);
        private_nh.param<double>("calib_pitch", calib_pitch, 0.0);
        private_nh.param<double>("calib_yaw", calib_yaw, 0.0);
        private_nh.param<double>("board_l", board_l, 0.3);

        private_nh.param<double>("box_A_lx", box_A_lx, 0.0);
        private_nh.param<double>("box_A_ly", box_A_ly, 0.0);
        private_nh.param<double>("box_A_lz", box_A_lz, 0.0);
        private_nh.param<double>("box_A_x", box_A_x, 0.0);
        private_nh.param<double>("box_A_y", box_A_y, 0.0);
        private_nh.param<double>("box_A_z", box_A_z, 0.0);
        private_nh.param<double>("box_A_qx", box_A_roll, 0.0);
        private_nh.param<double>("box_A_qy", box_A_pitch, 0.0);
        private_nh.param<double>("box_A_qz", box_A_yaw, 0.0);

        private_nh.param<double>("box_B_lx", box_B_lx, 0.0);
        private_nh.param<double>("box_B_ly", box_B_ly, 0.0);
        private_nh.param<double>("box_B_lz", box_B_lz, 0.0);
        private_nh.param<double>("box_B_x", box_B_x, 0.0);
        private_nh.param<double>("box_B_y", box_B_y, 0.0);
        private_nh.param<double>("box_B_z", box_B_z, 0.0);
        private_nh.param<double>("box_B_qx", box_B_roll, 0.0);
        private_nh.param<double>("box_B_qy", box_B_pitch, 0.0);
        private_nh.param<double>("box_B_qz", box_B_yaw, 0.0);
    }
};
#endif // LIDAR_CALIB_HEADER_H

