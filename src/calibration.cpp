#include "header.h"
#include "lidar_to_lidar_calibration/TestConfig.h"

class LidarCalibration : public ParamServer
{
public:

    ros::Subscriber source_lidar_sub; // Subscriber for source LiDAR data
    ros::Subscriber target_lidar_sub; // Subscriber for target camera data
    ros::Subscriber initialpose_sub; // Subscriber for initial transformation data
    ros::Subscriber header_sub; // Subscriber for initial pose data

    ros::Publisher merged_cloud_pub; // Publisher for merged point cloud
    ros::Publisher calib_result_pub; // Publisher for source LiDAR point clouds


    deque<PointCloudPtr> source_lidar_queue; // Queue for source LiDAR point clouds
    PointCloudPtr source_merge_cloud; // Merged point cloud for source LiDAR
    PointCloudPtr target_current_cloud; // Current point cloud for target camera

    string source_frame;
    string target_frame;


    double transformLidarToLidar[7];
    bool trigger_calculation = false;
    bool update_pose = false;
    double add_x = 0.0;
    double add_y = 0.0;
    double add_z = 0.0;
    double add_roll = 0.0;
    double add_pitch = 0.0;
    double add_yaw = 0.0;

    tf2_ros::TransformBroadcaster tf_broadcaster; // TF broadcaster for transformations

    boost::recursive_mutex reconfigure_mutex;
    dynamic_reconfigure::Server<lidar_to_lidar_calibration::TestConfig> server;
    dynamic_reconfigure::Server<lidar_to_lidar_calibration::TestConfig>::CallbackType f_c;

    LidarCalibration() : server(reconfigure_mutex)
    {
        // Initialize subscribers for source and target LiDAR topics
        source_lidar_sub = nh.subscribe(source_lidar_topic, 10, &LidarCalibration::sourceLidarCallback, this);
        target_lidar_sub = nh.subscribe(target_lidar_topic, 10, &LidarCalibration::targetLidarCallback, this);
        initialpose_sub = nh.subscribe(initial_topic, 10, &LidarCalibration::initialPoseCallback, this);
        header_sub = nh.subscribe(flag_topic, 10, &LidarCalibration::FlagCallback, this);

        merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(merged_cloud_topic, 10);
        calib_result_pub = nh.advertise<geometry_msgs::PoseStamped>(calibration_result_topic, 10);

        f_c = boost::bind(&LidarCalibration::dynamicReconfigureCallback, this, _1, _2);
        server.setCallback(f_c);

        allocateMemory();
    }

    void allocateMemory()
    {
        // Allocate memory for point clouds
        source_merge_cloud.reset(new PointCloud);
        target_current_cloud.reset(new PointCloud);
        // for (int i = 0; i < 6; ++i)
        // {
        //     transformLidarToLidar[i] = 0.0;
        // }
        // transformLidarToLidar[6] = 1.0;
        source_frame = "initial";
        target_frame = "initial";
        tf_broadcaster = tf2_ros::TransformBroadcaster(); // Initialize the TF broadcaster
    }

    void eulor_to_q()
    {
        double roll = calib_roll * M_PI / 180.0;
        double pitch = calib_pitch * M_PI / 180.0;
        double yaw = calib_yaw * M_PI / 180.0;
        Eigen::Quaterniond q;
        // q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        //     * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        //     * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        transformLidarToLidar[0] = calib_x;
        transformLidarToLidar[1] = calib_y;
        transformLidarToLidar[2] = calib_z;
        transformLidarToLidar[3] = q.x();
        transformLidarToLidar[4] = q.y();
        transformLidarToLidar[5] = q.z();
        transformLidarToLidar[6] = q.w();
    }

    void sourceLidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        PointCloudPtr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud);
        source_frame = msg->header.frame_id; // Update source frame from message header
        source_lidar_queue.push_back(cloud);

        if (source_lidar_queue.size() > source_lidar_queue_size)
        {
            source_lidar_queue.pop_front();
        }
        source_merge_cloud = mergePointClouds(source_lidar_queue);
        sensor_msgs::PointCloud2 merged_msg;
        pcl::toROSMsg(*source_merge_cloud, merged_msg);
        merged_msg.header = msg->header; // Set the header of the merged point cloud
        merged_cloud_pub.publish(merged_msg);
    }
    void targetLidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg, *target_current_cloud);
        target_frame = msg->header.frame_id; // Update target frame from message header
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        // Extract the transformation from the initial pose message
        transformLidarToLidar[0] = msg->pose.pose.position.x;
        transformLidarToLidar[1] = msg->pose.pose.position.y;
        transformLidarToLidar[2] = msg->pose.pose.position.z;

        transformLidarToLidar[3] = msg->pose.pose.orientation.x;
        transformLidarToLidar[4] = msg->pose.pose.orientation.y;
        transformLidarToLidar[5] = msg->pose.pose.orientation.z;
        transformLidarToLidar[6] = msg->pose.pose.orientation.w;
        ROS_INFO("Calibration started");
        Computer_calib();
    }
    void FlagCallback(const std_msgs::HeaderConstPtr& msg)
    {
        if (msg->frame_id == "calibration_start")
        {
            ROS_INFO("Calibration started");
            Computer_calib();
        }
    }

    void Computer_calib()
    {
        if (source_merge_cloud->empty() || target_current_cloud->empty())
        {
            ROS_WARN("Source or target point cloud is empty, skipping calibration.");
            return;
        }
        // ndt初配准
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setResolution(1.0);
        ndt.setMaximumIterations(50);
        ndt.setInputSource(target_current_cloud);
        ndt.setInputTarget(source_merge_cloud);
        Eigen::Matrix4f initial_guess = convertToEigenMatrix4f(transformLidarToLidar);
        PointCloudPtr aligned_cloud(new PointCloud);
        ndt.align(*aligned_cloud, initial_guess);

        // icp配准
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations(200);  // 迭代次数
        icp.setMaxCorrespondenceDistance(0.2); // 最大对应点距离
        icp.setTransformationEpsilon(1e-6);   // 收敛阈值
        icp.setEuclideanFitnessEpsilon(1e-6); // 欧氏距离阈值
        icp.setRANSACIterations(false);  // 不使用RANSAC
        icp.setInputSource(target_current_cloud);
        icp.setInputTarget(source_merge_cloud);
        PointCloudPtr icp_aligned_cloud(new PointCloud);
        icp.align(*icp_aligned_cloud, ndt.getFinalTransformation());
        printf("ICP score: %f\n", icp.getFitnessScore());
        Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
        if (icp.getFitnessScore() <= icp_score_threshold)
        {
            Eigen::Vector3f icp_translation = final_transformation.block<3, 1>(0, 3);
            Eigen::Matrix3f icp_rotation = final_transformation.block<3, 3>(0, 0);
            Eigen::Quaternionf icp_quaternion(icp_rotation);
            transformLidarToLidar[0] = static_cast<double>(icp_translation.x());
            transformLidarToLidar[1] = static_cast<double>(icp_translation.y());
            transformLidarToLidar[2] = static_cast<double>(icp_translation.z());
            transformLidarToLidar[3] = static_cast<double>(icp_quaternion.x());
            transformLidarToLidar[4] = static_cast<double>(icp_quaternion.y());
            transformLidarToLidar[5] = static_cast<double>(icp_quaternion.z());
            transformLidarToLidar[6] = static_cast<double>(icp_quaternion.w());
        }
        else
        {
            ROS_WARN("ICP score exceeds threshold, calibration failed.");
            // return;
        }
        updateParam(); // Update dynamic reconfigure parameters

    }

    void updateParam()
    {
        lidar_to_lidar_calibration::TestConfig config;
        config.calib_x = transformLidarToLidar[0];
        config.calib_y = transformLidarToLidar[1];
        config.calib_z = transformLidarToLidar[2];
        Eigen::Quaterniond quaternion_result(transformLidarToLidar[6], transformLidarToLidar[3], transformLidarToLidar[4], transformLidarToLidar[5]);
        Eigen::Vector3d euler_result = quaternion_result.toRotationMatrix().eulerAngles(2, 1, 0); // Yaw, Pitch, Roll
        config.calib_yaw = euler_result[0] * 180.0 / M_PI; // Convert to degrees
        config.calib_pitch = euler_result[1] * 180.0 / M_PI; // Convert to degrees
        config.calib_roll = euler_result[2] * 180.0 / M_PI; // Convert to degrees
        trigger_calculation = false; // Reset the trigger for next calculation
        config.trigger_calculation = false; // Reset the trigger for next calculation

        config.add_x = 0.0;
        config.add_y = 0.0;
        config.add_z = 0.0;
        config.add_roll = 0.0;
        config.add_pitch = 0.0;
        config.add_yaw = 0.0;
        update_pose = false;
        server.updateConfig(config); // Update dynamic reconfigure parameters
        ROS_INFO("Dynamic reconfigure parameters updated.");

    }

    Eigen::Matrix4f convertToEigenMatrix4f(const double trans[7])
    {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::Vector3d translation(trans[0],trans[1],trans[2]);
        Eigen::Quaterniond quaternion(trans[6],trans[3],trans[4],trans[5]);
        Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
        mat.block<3, 3>(0, 0) = rotation;
        mat.block<3, 1>(0, 3) = translation;
        return mat.cast<float>();
    }

    void add_pose()
    {
        double add_trans[7];
        double roll = add_roll * M_PI / 180.0;
        double pitch = add_pitch * M_PI / 180.0;
        double yaw = add_yaw * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf2::Transform add_mat;
        add_mat.setOrigin(tf2::Vector3(add_x,add_y,add_z));
        add_mat.setRotation(q);
        tf2::Transform current_mat;
        current_mat.setOrigin(tf2::Vector3(transformLidarToLidar[0],transformLidarToLidar[1],transformLidarToLidar[2]));
        current_mat.setRotation(tf2::Quaternion(transformLidarToLidar[3],transformLidarToLidar[4],transformLidarToLidar[5],transformLidarToLidar[6]));
        tf2::Transform new_matrix = current_mat * add_mat;

        transformLidarToLidar[0] = new_matrix.getOrigin().x();
        transformLidarToLidar[1] = new_matrix.getOrigin().y();
        transformLidarToLidar[2] = new_matrix.getOrigin().z();
        transformLidarToLidar[3] = new_matrix.getRotation().x();
        transformLidarToLidar[4] = new_matrix.getRotation().y();
        transformLidarToLidar[5] = new_matrix.getRotation().z();
        transformLidarToLidar[6] = new_matrix.getRotation().w();

        // Eigen::Quaterniond q;
        // q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        //     * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        //     * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        // add_trans[0] = add_x;
        // add_trans[1] = add_y;
        // add_trans[2] = add_z;
        // add_trans[3] = q.x();
        // add_trans[4] = q.y();
        // add_trans[5] = q.z();
        // add_trans[6] = q.w();
        // Eigen::Matrix4f add_matrix = convertToEigenMatrix4f(add_trans);
        // Eigen::Matrix4f current_matrix = convertToEigenMatrix4f(transformLidarToLidar);
        // Eigen::Matrix4f new_matrix = current_matrix * add_matrix; // Combine the transformations

        // Eigen::Vector3f new_translation = new_matrix.block<3, 1>(0, 3);
        // Eigen::Matrix3f new_rotation = new_matrix.block<3, 3>(0, 0);
        // Eigen::Quaternionf new_quaternion(new_rotation);
        // transformLidarToLidar[0] = static_cast<double>(new_translation.x());
        // transformLidarToLidar[1] = static_cast<double>(new_translation.y());
        // transformLidarToLidar[2] = static_cast<double>(new_translation.z());
        // transformLidarToLidar[3] = static_cast<double>(new_quaternion.x());
        // transformLidarToLidar[4] = static_cast<double>(new_quaternion.y());
        // transformLidarToLidar[5] = static_cast<double>(new_quaternion.z());
        // transformLidarToLidar[6] = static_cast<double>(new_quaternion.w());

        
    }

    PointCloudPtr mergePointClouds(const deque<PointCloudPtr>& queue)
    {
        PointCloudPtr merged_cloud(new PointCloud);
        for (const auto& cloud : queue)
        {
            *merged_cloud += *cloud; // Merge point clouds
        }
        return merged_cloud;
    }

    void dynamicReconfigureCallback(lidar_to_lidar_calibration::TestConfig &config, uint32_t level)
    {
        // Update calibration parameters from dynamic reconfigure
        calib_x = config.calib_x;
        calib_y = config.calib_y;
        calib_z = config.calib_z;
        calib_roll = config.calib_roll;
        calib_pitch = config.calib_pitch;
        calib_yaw = config.calib_yaw;
        trigger_calculation = config.trigger_calculation;
        add_x = config.add_x;
        add_y = config.add_y;
        add_z = config.add_z;
        add_roll = config.add_roll;
        add_pitch = config.add_pitch;
        add_yaw = config.add_yaw;
        icp_score_threshold = config.icp_score_threshold;

        eulor_to_q(); // Convert Euler angles to quaternion
        if(add_x != 0.0 || add_y != 0.0 || add_z != 0.0 ||
           add_roll != 0.0 || add_pitch != 0.0 || add_yaw != 0.0)
        {
            add_pose();
            update_pose = true;
        }
        ROS_INFO("Dynamic reconfigure parameters updated: "
                 "calib_x = %f, calib_y = %f, calib_z = %f, "
                 "calib_roll = %f, calib_pitch = %f, calib_yaw = %f"
                 "add_x = %f, add_y = %f, add_z = %f, "
                 "add_roll = %f, add_pitch = %f, add_yaw = %f, ",
                 calib_x, calib_y, calib_z,
                 calib_roll, calib_pitch, calib_yaw,
                 add_x, add_y, add_z,
                 add_roll, add_pitch, add_yaw);
    }


    void publishTF()
    {
        ros::Rate rate(10); // Publish at 10 Hz
        while (ros::ok())
        {
            if (!source_merge_cloud->empty() && !target_current_cloud->empty())
            {
                // Create a TransformStamped message
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = source_frame;
                transformStamped.child_frame_id = target_frame;

                transformStamped.transform.translation.x = transformLidarToLidar[0];
                transformStamped.transform.translation.y = transformLidarToLidar[1];
                transformStamped.transform.translation.z = transformLidarToLidar[2];

                transformStamped.transform.rotation.x = transformLidarToLidar[3];
                transformStamped.transform.rotation.y = transformLidarToLidar[4];
                transformStamped.transform.rotation.z = transformLidarToLidar[5];
                transformStamped.transform.rotation.w = transformLidarToLidar[6];


                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header = transformStamped.header;
                poseStamped.pose.position.x = transformLidarToLidar[0];
                poseStamped.pose.position.y = transformLidarToLidar[1];
                poseStamped.pose.position.z = transformLidarToLidar[2];
                poseStamped.pose.orientation.x = transformLidarToLidar[3];
                poseStamped.pose.orientation.y = transformLidarToLidar[4];
                poseStamped.pose.orientation.z = transformLidarToLidar[5];
                poseStamped.pose.orientation.w = transformLidarToLidar[6];
                // Publish the pose
                calib_result_pub.publish(poseStamped);

                // Publish the transformation
                tf_broadcaster.sendTransform(transformStamped);
            }
            if (update_pose)
            {
                ros::Duration duration(0.1);
                updateParam();
            }

            if (trigger_calculation)
            {
                ROS_INFO("Triggering calibration calculation.");
                Computer_calib();
            }
            rate.sleep();
        }
    }
};




int main (int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");

    LidarCalibration LC;

    ROS_INFO("\033[1;32m%s\033[0m", "Lidar Calibration Node Started");

    std::thread publishtfthread(&LidarCalibration::publishTF, &LC);

    ros::spin();
    publishtfthread.join();
    return 0;
}