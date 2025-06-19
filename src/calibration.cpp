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
    ros::Publisher target_cloud_repub;
    ros::Publisher calib_result_pub; // Publisher for source LiDAR point clouds
    ros::Publisher pub_mark_A;
    ros::Publisher pub_mark_B;
    ros::Publisher boundary_pub;
    ros::Publisher centrol;
    ros::Publisher mesh_pub;

    deque<PointCloudPtr> source_lidar_queue; // Queue for source LiDAR point clouds
    deque<PointXYZIRTCloudPtr> target_lidar_queue; // Queue for source LiDAR point clouds
    PointCloudPtr source_merge_cloud; // Merged point cloud for source LiDAR
    PointXYZIRTCloudPtr target_current_cloud; // Current point cloud for target camera

    PointCloudPtr source_raw_cloud;
    PointXYZIRTCloudPtr target_raw_cloud;

    PointCloudPtr source_center_cloud;
    PointCloudPtr target_center_cloud;

    PointType centroid_target;
    PointType centroid_source;

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

        // merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(merged_cloud_topic, 10);
        merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + source_lidar_topic + "/merge_cloud", 10);
        target_cloud_repub = nh.advertise<sensor_msgs::PointCloud2>(target_lidar_topic + "/filter", 10);
        boundary_pub = nh.advertise<sensor_msgs::PointCloud2>(target_lidar_topic + "_mesh", 10);
        center_A_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + source_lidar_topic + "/center", 10);
        mesh_pub = nh.advertise<sensor_msgs::PointCloud2>(target_lidar_topic + "_edge", 10);
        calib_result_pub = nh.advertise<geometry_msgs::PoseStamped>(calibration_result_topic, 10);
        pub_mark_A = nh.advertise<visualization_msgs::Marker>(marker_A_topic, 10);
        pub_mark_B = nh.advertise<visualization_msgs::Marker>(marker_B_topic, 10);
        f_c = boost::bind(&LidarCalibration::dynamicReconfigureCallback, this, _1, _2);
        server.setCallback(f_c);

        allocateMemory();
    }

    void allocateMemory()
    {
        // Allocate memory for point clouds
        source_merge_cloud.reset(new PointCloud);
        target_current_cloud.reset(new PointXYZIRTCloud);
        source_raw_cloud.reset(new PointCloud);
        target_raw_cloud.reset(new PointXYZIRTCloud);
        source_center_cloud.reset(new PointCloud);
        target_center_cloud.reset(new PointCloud);
        // for (int i = 0; i < 6; ++i)
        // {
        //     transformLidarToLidar[i] = 0.0;
        // }
        // transformLidarToLidar[6] = 1.0;
        source_frame = "initial";
        target_frame = "initial";
        tf_broadcaster = tf2_ros::TransformBroadcaster(); // Initialize the TF broadcaster
    }

    // 更新transformLidarToLidar
    void update_transformLidarToLidar()
    {
        double roll = calib_roll * M_PI / 180.0;
        double pitch = calib_pitch * M_PI / 180.0;
        double yaw = calib_yaw * M_PI / 180.0;
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
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

    // Eigen 欧拉角(角度值)转四元数
    Eigen::Quaterniond eulor_deg_to_q(double roll_deg, double pitch_deg, double yaw_deg)
    {
        double roll = roll_deg * M_PI / 180.0;
        double pitch = pitch_deg * M_PI / 180.0;
        double yaw = yaw_deg * M_PI / 180.0;
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        return q;
    }

    // 发布点云的函数
    void publishPointCloudMsg(const ros::Publish& publisher,
                            const std_msgs::Header& header,
                            const PointCloudPtr output_cloud)
    {
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header = header;
        output_msg.header.stamp = ros::Time::now();
        publisher.publish(output_msg);
    }

    // 重载发布点云的函数
    void publishPointCloudMsg(const ros::Publish& publisher,
                            const std_msgs::Header& header,
                            const PointXYZIRTCloudPtr output_cloud)
    {
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header = header;
        output_msg.header.stamp = ros::Time::now();
        publisher.publish(output_msg);
    }

    void sourceLidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        source_frame = msg->header.frame_id;
        PointCloudPtr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud); // msg转PointCloudPtr
        source_raw_cloud = cloud;

        // 箱体滤波，保留 箱体内的点云为 cloud_filtered
        PointCloudPtr cloud_filtered(new PointCloud);
        Eigen::Vector3d translation(box_A_x, box_A_y, box_A_z);
        Eigen::Quaterniond quaternion = eulor_deg_to_q(box_A_roll, box_A_pitch, box_A_yaw);
        Eigen::Matrix4f T = convertToEigenMatrix4f(translation, quaternion);
        Eigen::Matrix4f T_inv = T.inverse();
        for (const auto& pt : cloud->points)
        {
            Eigen::Vector4f p_sensor(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f p_cube = T_inv * p_sensor;
            if ( fabs(p_cube.x()) <= box_A_lx/2.0f &&
                 fabs(p_cube.y()) <= box_A_ly/2.0f &&
                 fabs(p_cube.z()) <= box_A_lz/2.0f )
            {
                cloud_filtered->points.push_back(pt);
            }
        }

        // 将点云添加到队列中，并融合点云为 source_merge_cloud
        source_lidar_queue.push_back(cloud_filtered);
        if (source_lidar_queue.size() > source_lidar_queue_size)
        {
            source_lidar_queue.pop_front();
        }
        source_merge_cloud = mergePointClouds(source_lidar_queue);

        // 发布融合后的点云
        // sensor_msgs::PointCloud2 merged_msg;
        // pcl::toROSMsg(*source_merge_cloud, merged_msg);
        // merged_msg.header = msg->header;
        // merged_msg.header.stamp = ros::Time::now();
        // merged_cloud_pub.publish(merged_msg);
        publishPointCloudMsg(merged_cloud_pub, msg->header, source_merge_cloud);

        // 将点云进行过滤后提取中心点，并发布出去 plan_cloud
        PointCloudPtr plan_cloud(new PointCloud);
        if (!source_merge_cloud->empty())
        {
            pcl::VoxelGrid<PointType> voxel_filter;
            voxel_filter.setInputCloud(source_merge_cloud);
            voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
            PointCloudPtr filtered_cloud(new PointCloud);
            voxel_filter.filter(*filtered_cloud);

            float sum_x = 0.0f;
            float sum_y = 0.0f;
            float sum_z = 0.0f;
            for (const auto& pt : filtered_cloud->points)
            {
                sum_x += pt.x;
                sum_y += pt.y;
                sum_z += pt.z;
            }
            size_t num_points = filtered_cloud->points.size();
            centroid_source.x = sum_x / num_points;
            centroid_source.y = sum_y / num_points;
            centroid_source.z = sum_z / num_points;
            centroid_source.intensity = 10.0f;
            plan_cloud->points.push_back(centroid_source);
        }

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*plan_cloud, output_msg);
        output_msg.header = msg->header;
        output_msg.header.stamp = ros::Time::now();
        center_A_pub.publish(output_msg);


    }
    void targetLidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        
        // pcl::fromROSMsg(*msg, *target_current_cloud);
        // target_frame = msg->header.frame_id; // Update target frame from message header
        PointXYZIRTCloudPtr cloud(new PointXYZIRTCloud);
        pcl::fromROSMsg(*msg, *cloud);
        target_raw_cloud = cloud;
        target_frame = msg->header.frame_id; // Update target frame from message header
        target_lidar_queue.push_back(target_raw_cloud);
        if (target_lidar_queue.size() > source_lidar_queue_size)
        {
            target_lidar_queue.pop_front();
        }
        PointXYZIRTCloudPtr target_merge_cloud = mergePointClouds(target_lidar_queue);


        Eigen::Vector3d translation(box_B_x, box_B_y, box_B_z);
        Eigen::Quaterniond quaternion = eulor_deg_to_q(box_B_roll, box_B_pitch, box_B_yaw);
        Eigen::Matrix4f T = convertToEigenMatrix4f(translation, quaternion);
        Eigen::Matrix4f T_inv = T.inverse();
        target_current_cloud->points.clear();
        for (const auto& pt : target_merge_cloud->points)
        {
            Eigen::Vector4f p_sensor(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f p_cube = T_inv * p_sensor;
            if ( fabs(p_cube.x()) <= box_B_lx/2.0f &&
                 fabs(p_cube.y()) <= box_B_ly/2.0f &&
                 fabs(p_cube.z()) <= box_B_lz/2.0f )
            {
                target_current_cloud->points.push_back(pt);
            }
        }
        //
        std::map<uint16_t, std::pair<PointXYZIRT, PointXYZIRT>> ring_boundary_points;
        for (const auto& point : target_current_cloud->points)
        {
            uint16_t ring = point.ring;
            if (ring_boundary_points.find(ring) == ring_boundary_points.end())
            {
                ring_boundary_points[ring] = std::make_pair(point, point);
            }
            else
            {
                auto& left_point = ring_boundary_points[ring].first;
                auto& right_point = ring_boundary_points[ring].second;
                if (point.x < left_point.x)
                {
                    left_point = point;
                }
                if (point.x > right_point.x)
                {
                    right_point = point;
                }
            }
        }

        std::vector<uint16_t> rings;
        for (const auto& kv : ring_boundary_points)
        {
            rings.push_back(kv.first);
        }
        std::sort(rings.begin(), rings.end());
        PointCloudPtr group(new PointCloud);
        PointCloudPtr edge_cloud(new PointCloud);
        float prev_x = 0.0f;
        float prev_y = 0.0f;
        float prev_z = 0.0f;
        for (size_t i = 0; i < rings.size(); ++i)
        {
            uint16_t ring = rings[i];
            auto& left_point = ring_boundary_points[ring].first;
            auto& right_point = ring_boundary_points[ring].second;
            edge_cloud->points.push_back(convertVelodyneToXYZI(left_point));
            edge_cloud->points.push_back(convertVelodyneToXYZI(right_point));
            float dx = (left_point.x + right_point.x) / 2;
            float dy = (left_point.y + right_point.y) / 2;
            float dz = (left_point.z + right_point.z) / 2;
            prev_x += dx;
            prev_y += dy;
            prev_z += dz;
        }
        prev_x /= rings.size();
        prev_y /= rings.size();
        prev_z /= rings.size();
        float dis_l = board_l / std::sqrt(2);
        float step = 0.002f;  // 0.5cm
        float point_start = 0.0f;
        float point_end = dis_l;
        for (float y = point_start; y <= point_end; y += step)
        {
            PointType pt1, pt2, pt3, pt4;
            pt1.x = prev_x;
            pt2.x = prev_x;
            pt3.x = prev_x;
            pt4.x = prev_x;

            pt1.y = y + prev_y - dis_l;
            pt2.y = y + prev_y;
            pt3.y = prev_y + dis_l - y;
            pt4.y = prev_y - y;

            pt1.z = prev_z + y;
            pt2.z = prev_z + dis_l - y;
            pt3.z = prev_z - y;
            pt4.z = prev_z - dis_l + y;

            pt1.intensity = 255.0f;
            pt2.intensity = 255.0f;
            pt3.intensity = 255.0f;
            pt4.intensity = 255.0f;

            group->points.push_back(pt1);
            group->points.push_back(pt2);
            group->points.push_back(pt3);
            group->points.push_back(pt4);
        }

        if (!edge_cloud->empty() && !group->empty())
        {
            pcl::NormalDistributionsTransform<PointType, PointType> ndt;
            ndt.setTransformationEpsilon(0.01);
            ndt.setResolution(0.1f);
            ndt.setMaximumIterations(50);
            ndt.setInputSource(edge_cloud);
            ndt.setInputTarget(group);
            Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
            PointCloudPtr aligned_cloud(new PointCloud);
            ndt.align(*aligned_cloud, initial_guess);

            // icp配准
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaximumIterations(100);  // 迭代次数
            icp.setMaxCorrespondenceDistance(0.2); // 最大对应点距离
            icp.setTransformationEpsilon(1e-6);   // 收敛阈值
            icp.setEuclideanFitnessEpsilon(1e-6); // 欧氏距离阈值
            icp.setRANSACIterations(false);  // 不使用RANSAC
            icp.setInputSource(edge_cloud);
            icp.setInputTarget(group);
            PointCloudPtr icp_aligned_cloud(new PointCloud);
            icp.align(*icp_aligned_cloud, ndt.getFinalTransformation());
            // printf("ICP score: %f\n", icp.getFitnessScore());
            Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
            if (icp.getFitnessScore() <= 0.001)
            {
                PointCloudPtr transformed_group(new PointCloud);
                pcl::transformPointCloud(*group, *transformed_group, final_transformation);
                *group = *transformed_group;
            }
            else
            {
                printf("ICP score: %f\n", icp.getFitnessScore());
                ROS_WARN("ICP score exceeds threshold, calibration failed.");
            }

            //
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            float sum_z = 0.0f;
            for (const auto& pt : group->points)
            {
                sum_x += pt.x;
                sum_y += pt.y;
                sum_z += pt.z;
            }
            size_t num_points = group->points.size();
            centroid_target.x = sum_x / num_points;
            centroid_target.y = sum_y / num_points;
            centroid_target.z = sum_z / num_points;
            centroid_target.intensity = 10.0f;
            group->points.push_back(centroid_target);
        }



        // PointXYZIRTCloudPtr boundary_cloud(new PointXYZIRTCloud);
        // for (const auto& kv : ring_boundary_points)
        // {
        //     boundary_cloud->points.push_back(kv.second.first);
        //     boundary_cloud->points.push_back(kv.second.second);
        // }
        sensor_msgs::PointCloud2 output_msg;
        // pcl::toROSMsg(*boundary_cloud, output_msg);
        
        pcl::toROSMsg(*group, output_msg);
        output_msg.header = msg->header;
        output_msg.header.stamp = ros::Time::now();
        boundary_pub.publish(output_msg);

        sensor_msgs::PointCloud2 edge_msg;
        pcl::toROSMsg(*edge_cloud, edge_msg);
        edge_msg.header = msg->header;
        edge_msg.header.stamp = ros::Time::now();

        mesh_pub.publish(edge_msg);

        //
        sensor_msgs::PointCloud2 merged_msg;
        pcl::toROSMsg(*target_current_cloud, merged_msg);
        merged_msg.header = msg->header;
        merged_msg.header.stamp = ros::Time::now();
        target_cloud_repub.publish(merged_msg);
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
            // Computer_calib();
            source_center_cloud->points.clear();
            target_center_cloud->points.clear();
        }
        else if (msg->frame_id == "calibration_record_points")
        {
            // source_center_cloud->points.push_back(centroid_source);
            // target_center_cloud->points.push_back(centroid_target);
            // test

            PointType pt1a,pt1b,pt2a,pt2b,pt3a,pt3b,pt4a,pt4b,pt5a,pt5b;
            pt1a.x = 1.1;
            pt1a.y = 2.7;
            pt1a.z = 5.9;
            pt1b.x = -1.96905;
            pt1b.y = 3.0694702;
            pt1b.z = 4.32063;
            source_center_cloud->points.push_back(pt1a);
            target_center_cloud->points.push_back(pt1b);

            pt2a.x = 3.4;
            pt2a.y = 1.0;
            pt2a.z = 0.2;
            pt2b.x = 2.9981243;
            pt2b.y = 0.48969842;
            pt2b.z = 1.26417786;
            source_center_cloud->points.push_back(pt2a);
            target_center_cloud->points.push_back(pt2b);

            pt3a.x = 7.9;
            pt3a.y = 3.8;
            pt3a.z = 2.1;
            pt3b.x = 5.8514683;
            pt3b.y = 3.625053;
            pt3b.z = 4.9693063;
            source_center_cloud->points.push_back(pt3a);
            target_center_cloud->points.push_back(pt3b);

            pt4a.x = 4.3;
            pt4a.y = 6.7;
            pt4a.z = 9.3;
            pt4b.x = -0.93137337;
            pt4b.y = 7.6165165;
            pt4b.z = 8.326579;
            source_center_cloud->points.push_back(pt4a);
            target_center_cloud->points.push_back(pt4b);

            pt5a.x = 0.4;
            pt5a.y = 5.6;
            pt5a.z = 3.2;
            pt5b.x = -0.780418226;
            pt5b.y = 5.4775840;
            pt5b.z = 1.324280886;
            source_center_cloud->points.push_back(pt5a);
            target_center_cloud->points.push_back(pt5b);
        }
        else if (msg->frame_id == "calibration_computer")
        {
            estimateRigidTransformationAndPublish(source_center_cloud, target_center_cloud);
        }
    }

    PointType convertVelodyneToXYZI(const PointXYZIRT& pt)
    {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        return p;
    }

    void Computer_calib()
    {
        if (source_merge_cloud->empty() || target_current_cloud->empty())
        {
            ROS_WARN("Source or target point cloud is empty, skipping calibration.");
            return;
        }
        //
        PointCloudPtr target_current_cloud_new(new PointCloud);
        target_current_cloud_new->points.resize(target_current_cloud->points.size());
        for (size_t i = 0; i < target_current_cloud->points.size(); ++i)
        {
            const auto& pt_src = target_current_cloud->points[i];
            PointType pt_dst;
            pt_dst.x = pt_src.x;
            pt_dst.y = pt_src.y;
            pt_dst.z = pt_src.z;
            pt_dst.intensity = pt_src.intensity;
            target_current_cloud_new->points[i] = pt_dst;
        }


        // ndt初配准
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setResolution(1.0);
        ndt.setMaximumIterations(50);
        ndt.setInputSource(target_current_cloud_new);
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
        icp.setInputSource(target_current_cloud_new);
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
        config.icp_score_threshold = icp_score_threshold;

        config.add_x = 0.0;
        config.add_y = 0.0;
        config.add_z = 0.0;
        config.add_roll = 0.0;
        config.add_pitch = 0.0;
        config.add_yaw = 0.0;

        config.box_A_lx = box_A_lx;
        config.box_A_ly = box_A_ly;
        config.box_A_lz = box_A_lz;
        config.box_A_x = box_A_x;
        config.box_A_y = box_A_y;
        config.box_A_z = box_A_z;
        config.box_A_roll = box_A_roll;
        config.box_A_pitch = box_A_pitch;
        config.box_A_yaw = box_A_yaw;

        config.box_B_lx = box_B_lx;
        config.box_B_ly = box_B_ly;
        config.box_B_lz = box_B_lz;
        config.box_B_x = box_B_x;
        config.box_B_y = box_B_y;
        config.box_B_z = box_B_z;
        config.box_B_roll = box_B_roll;
        config.box_B_pitch = box_B_pitch;
        config.box_B_yaw = box_B_yaw;
        config.board_l = board_l;
        update_pose = false;
        server.updateConfig(config); // Update dynamic reconfigure parameters
        ROS_INFO("Dynamic reconfigure parameters updated.");

    }

    double computeEuclideanDistance(const PointType &point1, const PointType &point2)
    {
        return std::sqrt(
            std::pow(point1.x - point2.x, 2) +
            std::pow(point1.y - point2.y, 2) +
            std::pow(point1.z - point2.z, 2));
    }

    double computeRMSE(const PointCloudPtr &cloud1,
                       const PointCloudPtr &cloud2)
    {
        double sum_squared_error = 0.0;
        for (size_t i = 0; i < cloud1->size(); ++i)
        {
            double distance = computeEuclideanDistance(cloud1->points[i], cloud2->points[i]);
            sum_squared_error += distance * distance;
        }
        return std::sqrt(sum_squared_error / cloud1->size());
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

    Eigen::Matrix4f convertToEigenMatrix4f(Eigen::Vector3d translation, Eigen::Quaterniond quaternion)
    {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
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
        
    }
    void estimateRigidTransformationAndPublish(
        const PointCloudPtr &matched_scan_cloud,
        const PointCloudPtr &matched_map_cloud)
    {
        // 估计刚性变换
        pcl::registration::TransformationEstimationSVD<PointType, PointType> transformation_estimation;
        Eigen::Matrix4f transformation_matrix;
        transformation_estimation.estimateRigidTransformation(*matched_scan_cloud, *matched_map_cloud, transformation_matrix);

        // 评估刚体变换效果
        PointCloudPtr transformed_scan_cloud(new PointCloud);
        pcl::transformPointCloud(*matched_scan_cloud, *transformed_scan_cloud, transformation_matrix);
        double transformed_rmse = computeRMSE(transformed_scan_cloud, matched_map_cloud);
        std::cout << "Transformed RMSE: " << transformed_rmse << std::endl;
        if (transformed_rmse > 2.0)
        {
            std::cout<<"Transformation does not meet threshold!"<<std::endl;
            return;
        }
        Eigen::Matrix3d Rtrans = transformation_matrix.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond q_trans(Rtrans);
        Eigen::Vector3d t_trans = transformation_matrix.block<3, 1>(0, 3).cast<double>();
        std::cout << "q = " << std::endl;
        std::cout << q_trans.x() << " " << q_trans.y() << " " << q_trans.z() << " " << q_trans.w() << std::endl;
        std::cout << "t = " << std::endl;
        std::cout << t_trans[0] << " " << t_trans[1] << " " << t_trans[2] << std::endl;
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

    PointXYZIRTCloudPtr mergePointClouds(const deque<PointXYZIRTCloudPtr>& queue)
    {
        PointXYZIRTCloudPtr merged_cloud(new PointXYZIRTCloud);
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
        box_A_lx = config.box_A_lx;
        box_A_ly = config.box_A_ly;
        box_A_lz = config.box_A_lz;
        box_A_x = config.box_A_x;
        box_A_y = config.box_A_y;
        box_A_z = config.box_A_z;
        box_A_roll = config.box_A_roll;
        box_A_pitch = config.box_A_pitch;
        box_A_yaw = config.box_A_yaw;

        box_B_lx = config.box_B_lx;
        box_B_ly = config.box_B_ly;
        box_B_lz = config.box_B_lz;
        box_B_x = config.box_B_x;
        box_B_y = config.box_B_y;
        box_B_z = config.box_B_z;
        box_B_roll = config.box_B_roll;
        box_B_pitch = config.box_B_pitch;
        box_B_yaw = config.box_B_yaw;
        board_l = config.board_l;

        update_transformLidarToLidar();
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
    void publishCubeMarker(ros::Publisher& marker_pub,string frame_id,
        double lx, double ly, double lz, double x, double y, double z,
        double roll, double pitch, double yaw, const std::vector<double> &rgb)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cube";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.r = rgb[0];
        marker.color.g = rgb[1];
        marker.color.b = rgb[2];
        marker.color.a = 1.0;
        std::vector<Eigen::Vector3f> points_local = {
            {float(lx/2), float(ly/2), float(lz/2)},
            {float(-lx/2), float(ly/2), float(lz/2)},
            {float(-lx/2), float(-ly/2), float(lz/2)},
            {float(lx/2), float(-ly/2), float(lz/2)},
            {float(lx/2), float(ly/2), float(-lz/2)},
            {float(-lx/2), float(ly/2), float(-lz/2)},
            {float(-lx/2), float(-ly/2), float(-lz/2)},
            {float(lx/2), float(-ly/2), float(-lz/2)}
        };
        Eigen::Vector3d translation(x, y, z);
        Eigen::Quaterniond quaternion = eulor_deg_to_q(roll, pitch, yaw);
        Eigen::Matrix4f T = convertToEigenMatrix4f(translation, quaternion);
        std::vector<geometry_msgs::Point> vertices_world(8);
        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector4f p_local(points_local[i](0), points_local[i](1), points_local[i](2), 1.0f);
            Eigen::Vector4f p_world = T * p_local;
            geometry_msgs::Point pt;
            pt.x = p_world.x();
            pt.y = p_world.y();
            pt.z = p_world.z();
            vertices_world[i] = pt;
        }
        int edge_indices[24] = {
            0,1, 1,2, 2,3, 3,0,   // 前面4条边
            4,5, 5,6, 6,7, 7,4,   // 后面4条边
            0,4, 1,5, 2,6, 3,7    // 连接前后4条边
        };
        for (int i = 0; i < 24; i += 2)
        {
            marker.points.push_back(vertices_world[edge_indices[i]]);
            marker.points.push_back(vertices_world[edge_indices[i + 1]]);
        }
        marker_pub.publish(marker);

    }

    void publishTF()
    {
        ros::Rate rate(10); // Publish at 10 Hz
        while (ros::ok())
        {
            if (!source_raw_cloud->empty() && !target_raw_cloud->empty())
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
                vector<double> marker_A_rgb = {1.0, 0.0, 0.0};
                
                publishCubeMarker(pub_mark_A, source_frame, box_A_lx, box_A_ly, box_A_lz, box_A_x, box_A_y, box_A_z, box_A_roll, box_A_pitch, box_A_yaw, marker_A_rgb);
                vector<double> marker_B_rgb = {0.0, 1.0, 0.0};
                publishCubeMarker(pub_mark_B, target_frame, box_B_lx, box_B_ly, box_B_lz, box_B_x, box_B_y, box_B_z, box_B_roll, box_B_pitch, box_B_yaw, marker_B_rgb);
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