#include "header.h"
#include "lidar_to_lidar_calibration/TestConfig.h"

class LidarCalibration : public ParamServer
{
public:
    ros::Subscriber source_lidar_sub; // Subscriber for source LiDAR data
    ros::Subscriber target_lidar_sub; // Subscriber for target camera data
    ros::Subscriber initialpose_sub;  // Subscriber for initial transformation data
    ros::Subscriber header_sub;       // Subscriber for initial pose data

    ros::Publisher merged_cloud_pub; // Publisher for merged point cloud
    ros::Publisher target_filtered_pub;
    ros::Publisher calib_result_pub; // Publisher for source LiDAR point clouds
    ros::Publisher pub_mark_A;
    ros::Publisher pub_mark_B;
    ros::Publisher boundary_pub;
    ros::Publisher boundary_raw_pub;
    ros::Publisher center_A_pub;
    ros::Publisher all_centroid_A_pub;
    ros::Publisher all_centroid_B_pub;
    ros::Publisher edge_pub;
    ros::Publisher feedback_flag_pub;

    deque<PointCloudPtr> source_lidar_queue;       // Queue for source LiDAR point clouds
    deque<PointXYZIRTCloudPtr> target_lidar_queue; // Queue for source LiDAR point clouds
    PointCloudPtr source_merge_cloud;              // Merged point cloud for source LiDAR
    PointXYZIRTCloudPtr target_current_cloud;      // Current point cloud for target camera

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
    bool auto_find_center_source = false;
    bool auto_find_center_target = false;
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
        source_lidar_sub = nh.subscribe(source_lidar_topic, 10, &LidarCalibration::sourceLidarCallback, this);
        target_lidar_sub = nh.subscribe(target_lidar_topic, 10, &LidarCalibration::targetLidarCallback, this);
        initialpose_sub = nh.subscribe(initial_topic, 10, &LidarCalibration::initialPoseCallback, this);
        header_sub = nh.subscribe(flag_topic, 10, &LidarCalibration::FlagCallback, this);

        merged_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + source_lidar_topic + "/filtered", 10);
        target_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + target_lidar_topic + "/filtered", 10);
        boundary_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + target_lidar_topic + "/boundary_cloud", 10);
        boundary_raw_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + target_lidar_topic + "/boundary_raw_cloud", 10);
        center_A_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + source_lidar_topic + "/center", 10);
        edge_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration/" + target_lidar_topic + "/edge_cloud", 10);
        all_centroid_A_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration" + source_lidar_topic + "/all_centroid", 10);
        all_centroid_B_pub = nh.advertise<sensor_msgs::PointCloud2>("/calibration" + target_lidar_topic + "/all_centroid", 10);
        calib_result_pub = nh.advertise<geometry_msgs::PoseStamped>(calibration_result_topic, 10);
        pub_mark_A = nh.advertise<visualization_msgs::Marker>(marker_A_topic, 10);
        pub_mark_B = nh.advertise<visualization_msgs::Marker>(marker_B_topic, 10);
        feedback_flag_pub = nh.advertise<std_msgs::Header>(flag_topic, 10);
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
        q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        transformLidarToLidar[0] = calib_x;
        transformLidarToLidar[1] = calib_y;
        transformLidarToLidar[2] = calib_z;
        transformLidarToLidar[3] = q.x();
        transformLidarToLidar[4] = q.y();
        transformLidarToLidar[5] = q.z();
        transformLidarToLidar[6] = q.w();
    }

    void sourceLidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        source_frame = msg->header.frame_id;
        PointCloudPtr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud); // msg转PointCloudPtr
        source_raw_cloud = cloud;
        if (auto_find_center_source)
        {
            PointCloudPtr cloud2(new PointCloud);
            pcl::fromROSMsg(*msg, *cloud2);
            pcl::PointIndices::Ptr intensity_filtered_indices(new pcl::PointIndices);
            for (size_t i = 0; i < cloud2->points.size(); ++i)
            {
                if (cloud2->points[i].intensity > 150)
                {
                    intensity_filtered_indices->indices.push_back(i);
                    if (!intensity_filtered_indices->indices.empty())
                    {
                        PointCloudPtr intensity_filtered_cloud(new PointCloud);
                        pcl::ExtractIndices<PointType> extract;
                        extract.setInputCloud(cloud2);
                        extract.setIndices(intensity_filtered_indices);
                        extract.setNegative(false);
                        extract.filter(*intensity_filtered_cloud);

                        // 4. 聚类参数设置
                        std::vector<pcl::PointIndices> cluster_indices;
                        pcl::EuclideanClusterExtraction<PointType> ec;
                        ec.setClusterTolerance(0.5); // 设置聚类距离阈值，单位米，根据实际场景调整
                        ec.setMinClusterSize(5);     // 聚类点数阈值，根据需要调整
                        ec.setMaxClusterSize(10000);
                        ec.setInputCloud(intensity_filtered_cloud);
                        ec.extract(cluster_indices);

                        if (!cluster_indices.empty())
                        {
                            size_t max_cluster_idx = 0;
                            size_t max_points = 0;
                            for (size_t i = 0; i < cluster_indices.size(); ++i)
                            {
                                if (cluster_indices[i].indices.size() > max_points)
                                {
                                    max_points = cluster_indices[i].indices.size();
                                    max_cluster_idx = i;
                                }
                            }

                            // 6. 提取最大聚类簇点云
                            pcl::PointIndices::Ptr largest_cluster_indices(new pcl::PointIndices(cluster_indices[max_cluster_idx]));
                            PointCloudPtr largest_cluster_cloud(new PointCloud);
                            extract.setInputCloud(intensity_filtered_cloud);
                            extract.setIndices(largest_cluster_indices);
                            extract.filter(*largest_cluster_cloud);

                            // 7. 计算质心
                            Eigen::Vector4f centroid;
                            pcl::compute3DCentroid(*largest_cluster_cloud, centroid);
                            box_A_x = centroid[0];
                            box_A_y = centroid[1];
                            box_A_z = centroid[2];
                        }
                    }
                }
            }
            updateParam();
            auto_find_center_source = false;
        }

        // 箱体滤波，保留 箱体内的点云为 cloud_filtered
        PointCloudPtr cloud_filtered(new PointCloud);
        Eigen::Vector3d translation(box_A_x, box_A_y, box_A_z);
        Eigen::Quaterniond quaternion = eulor_deg_to_q(box_A_roll, box_A_pitch, box_A_yaw);
        Eigen::Matrix4f T = convertToEigenMatrix4f(translation, quaternion);
        Eigen::Matrix4f T_inv = T.inverse();
        for (const auto &pt : cloud->points)
        {
            Eigen::Vector4f p_sensor(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f p_cube = T_inv * p_sensor;
            if (fabs(p_cube.x()) <= box_A_lx / 2.0f &&
                fabs(p_cube.y()) <= box_A_ly / 2.0f &&
                fabs(p_cube.z()) <= box_A_lz / 2.0f)
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
            for (const auto &pt : filtered_cloud->points)
            {
                sum_x += pt.x;
                sum_y += pt.y;
                sum_z += pt.z;
            }
            size_t num_points = filtered_cloud->points.size();
            centroid_source.x = sum_x / num_points;
            centroid_source.y = sum_y / num_points;
            centroid_source.z = sum_z / num_points;
            centroid_source.intensity = 200.0f;
            plan_cloud->points.push_back(centroid_source);
        }

        // 发布中心点
        publishPointCloudMsg(center_A_pub, msg->header, plan_cloud);
        if (!source_center_cloud->points.empty())
        {
            publishPointCloudMsg(all_centroid_A_pub, msg->header, source_center_cloud);
        }

        vector<double> marker_A_rgb = {1.0, 0.0, 0.0};
        publishCubeMarker(pub_mark_A, source_frame, box_A_lx, box_A_ly, box_A_lz, box_A_x, box_A_y, box_A_z, box_A_roll, box_A_pitch, box_A_yaw, marker_A_rgb);
    }

    void targetLidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        target_frame = msg->header.frame_id;
        PointXYZIRTCloudPtr cloud(new PointXYZIRTCloud);
        pcl::fromROSMsg(*msg, *cloud);
        target_raw_cloud = cloud;
        if (auto_find_center_target)
        {
            PointCloudPtr cloud2(new PointCloud);
            pcl::fromROSMsg(*msg, *cloud2);
            pcl::PointIndices::Ptr intensity_filtered_indices(new pcl::PointIndices);
            for (size_t i = 0; i < cloud2->points.size(); ++i)
            {
                if (cloud2->points[i].intensity > 150)
                {
                    intensity_filtered_indices->indices.push_back(i);
                    if (!intensity_filtered_indices->indices.empty())
                    {
                        PointCloudPtr intensity_filtered_cloud(new PointCloud);
                        pcl::ExtractIndices<PointType> extract;
                        extract.setInputCloud(cloud2);
                        extract.setIndices(intensity_filtered_indices);
                        extract.setNegative(false);
                        extract.filter(*intensity_filtered_cloud);

                        // 4. 聚类参数设置
                        std::vector<pcl::PointIndices> cluster_indices;
                        pcl::EuclideanClusterExtraction<PointType> ec;
                        ec.setClusterTolerance(0.5); // 设置聚类距离阈值，单位米，根据实际场景调整
                        ec.setMinClusterSize(5);     // 聚类点数阈值，根据需要调整
                        ec.setMaxClusterSize(10000);
                        ec.setInputCloud(intensity_filtered_cloud);
                        ec.extract(cluster_indices);

                        if (!cluster_indices.empty())
                        {
                            size_t max_cluster_idx = 0;
                            size_t max_points = 0;
                            for (size_t i = 0; i < cluster_indices.size(); ++i)
                            {
                                if (cluster_indices[i].indices.size() > max_points)
                                {
                                    max_points = cluster_indices[i].indices.size();
                                    max_cluster_idx = i;
                                }
                            }

                            // 6. 提取最大聚类簇点云
                            pcl::PointIndices::Ptr largest_cluster_indices(new pcl::PointIndices(cluster_indices[max_cluster_idx]));
                            PointCloudPtr largest_cluster_cloud(new PointCloud);
                            extract.setInputCloud(intensity_filtered_cloud);
                            extract.setIndices(largest_cluster_indices);
                            extract.filter(*largest_cluster_cloud);

                            // 7. 计算质心
                            Eigen::Vector4f centroid;
                            pcl::compute3DCentroid(*largest_cluster_cloud, centroid);
                            box_B_x = centroid[0];
                            box_B_y = centroid[1];
                            box_B_z = centroid[2];
                        }
                    }
                }
            }
            updateParam();
            auto_find_center_target = false;
        }

        // 将点云添加到队列中，并融合点云为 target_merge_cloud
        target_lidar_queue.push_back(target_raw_cloud);
        if (target_lidar_queue.size() > source_lidar_queue_size)
        {
            target_lidar_queue.pop_front();
        }
        PointXYZIRTCloudPtr target_merge_cloud = mergePointClouds(target_lidar_queue);

        // 箱体滤波，保留 箱体内的点云为 target_current_cloud
        target_current_cloud->points.clear();
        Eigen::Vector3d translation(box_B_x, box_B_y, box_B_z);
        Eigen::Quaterniond quaternion = eulor_deg_to_q(box_B_roll, box_B_pitch, box_B_yaw);
        Eigen::Matrix4f T = convertToEigenMatrix4f(translation, quaternion);
        Eigen::Matrix4f T_inv = T.inverse();
        target_current_cloud->points.clear();
        for (const auto &pt : target_merge_cloud->points)
        {
            Eigen::Vector4f p_sensor(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f p_cube = T_inv * p_sensor;
            if (fabs(p_cube.x()) <= box_B_lx / 2.0f &&
                fabs(p_cube.y()) <= box_B_ly / 2.0f &&
                fabs(p_cube.z()) <= box_B_lz / 2.0f)
            {
                target_current_cloud->points.push_back(pt);
            }
        }

        // 将每一条线的左右端点保存到 ring_boundary_points
        std::map<uint16_t, std::pair<PointXYZIRT, PointXYZIRT>> ring_boundary_points;
        for (const auto &point : target_current_cloud->points)
        {
            uint16_t ring = point.ring;
            if (ring_boundary_points.find(ring) == ring_boundary_points.end())
            {
                ring_boundary_points[ring] = std::make_pair(point, point);
            }
            else
            {
                auto &left_point = ring_boundary_points[ring].first;
                auto &right_point = ring_boundary_points[ring].second;
                // 通过角度值找边界
                float left_angle = std::atan2(left_point.y, left_point.x);
                float right_angle = std::atan2(right_point.y, right_point.x);
                float this_angle = std::atan2(point.y, point.x);
                if (this_angle > left_angle)
                {
                    left_point = point;
                }
                else if (this_angle < right_angle)
                {
                    right_point = point;
                }

                // 缺乏横跨-180/180度线的解决方法，因实际用不到，暂时不处理
            }
        }

        std::vector<uint16_t> rings;
        for (const auto &kv : ring_boundary_points)
        {
            rings.push_back(kv.first);
        }
        std::sort(rings.begin(), rings.end());
        PointCloudPtr edge_cloud(new PointCloud); // 边界点

        // add -----------------------------
        PointCloudPtr group1(new PointCloud);
        PointCloudPtr group2(new PointCloud);
        PointCloudPtr group3(new PointCloud);
        PointCloudPtr group4(new PointCloud);
        double dis_point12 = 0.0;
        double last_point_dis = 0.0;
        bool start_lower_point = false;
        for (size_t i = 0; i < rings.size(); ++i)
        {
            uint16_t ring = rings[i];
            auto &left_point = ring_boundary_points[ring].first;
            auto &right_point = ring_boundary_points[ring].second;
            // edge_cloud->points.push_back(convertVelodyneToXYZI(left_point));
            // edge_cloud->points.push_back(convertVelodyneToXYZI(right_point));
            int left_inter, right_inter;
            if (!start_lower_point)
            {
                if (i == 0)
                {
                    group1->points.push_back(convertVelodyneToXYZI(left_point));
                    group4->points.push_back(convertVelodyneToXYZI(right_point));
                    left_inter = 10;
                    right_inter = 200;
                    last_point_dis = computeEuclideanDistance(left_point, right_point);
                }
                else if (i == 1)
                {
                    group1->points.push_back(convertVelodyneToXYZI(left_point));
                    group4->points.push_back(convertVelodyneToXYZI(right_point));
                    left_inter = 10;
                    right_inter = 200;
                    double this_point_dis = computeEuclideanDistance(left_point, right_point);
                    dis_point12 = this_point_dis - last_point_dis;
                    last_point_dis = this_point_dis;
                }
                else
                {
                    double this_point_dis = computeEuclideanDistance(left_point, right_point);
                    if ((this_point_dis - last_point_dis) >= 0.7 * dis_point12)
                    {
                        group1->points.push_back(convertVelodyneToXYZI(left_point));
                        group4->points.push_back(convertVelodyneToXYZI(right_point));
                        left_inter = 10;
                        right_inter = 200;
                    }
                    else
                    {
                        group2->points.push_back(convertVelodyneToXYZI(left_point));
                        group3->points.push_back(convertVelodyneToXYZI(right_point));
                        left_inter = 60;
                        right_inter = 120;
                        start_lower_point = true;
                    }
                }
            }
            else
            {
                group2->points.push_back(convertVelodyneToXYZI(left_point));
                group3->points.push_back(convertVelodyneToXYZI(right_point));
                left_inter = 60;
                right_inter = 120;
            }
            edge_cloud->points.push_back(convertVelodyneToXYZI(left_point, left_inter));
            edge_cloud->points.push_back(convertVelodyneToXYZI(right_point, right_inter));
        }
        // 计算中心点坐标
        float prev_x = 0.0f;
        float prev_y = 0.0f;
        float prev_z = 0.0f;
        PointCloudPtr group(new PointCloud);

        // 四个点坐标
        if (group1->points.size() > 1 && group2->points.size() > 1 && group3->points.size() > 1 && group4->points.size() > 1)
        {
            PointType up_point = fitLineAndGetIntersection(group1, group4);
            PointType left_point = fitLineAndGetIntersection(group1, group2);
            PointType right_point = fitLineAndGetIntersection(group3, group4);
            PointType lower_point = fitLineAndGetIntersection(group2, group3);

            prev_x = (up_point.x + left_point.x + right_point.x + lower_point.x) / 4;
            prev_y = (up_point.y + left_point.y + right_point.y + lower_point.y) / 4;
            prev_z = (up_point.z + left_point.z + right_point.z + lower_point.z) / 4;
            
            insertInterpolatnedPoints(up_point, left_point, 30, group);
            insertInterpolatnedPoints(left_point, lower_point, 30, group);
            insertInterpolatnedPoints(lower_point, right_point, 30, group);
            insertInterpolatnedPoints(right_point, up_point, 30, group);
            centroid_target.x = prev_x;
            centroid_target.y = prev_y;
            centroid_target.z = prev_z;
            centroid_target.intensity = 10.0f;
            group->points.push_back(centroid_target);
        }
        else
        {
            ROS_ERROR("Less than four rings, unable to calculate!");
        }
        

        // ---------------------------------
        
        // for (size_t i = 0; i < rings.size(); ++i)
        // {
        //     uint16_t ring = rings[i];
        //     auto &left_point = ring_boundary_points[ring].first;
        //     auto &right_point = ring_boundary_points[ring].second;
        //     edge_cloud->points.push_back(convertVelodyneToXYZI(left_point));
        //     edge_cloud->points.push_back(convertVelodyneToXYZI(right_point));
        //     float dx = (left_point.x + right_point.x) / 2;
        //     float dy = (left_point.y + right_point.y) / 2;
        //     float dz = (left_point.z + right_point.z) / 2;
        //     prev_x += dx;
        //     prev_y += dy;
        //     prev_z += dz;
        // }
        // prev_x /= rings.size();
        // prev_y /= rings.size();
        // // prev_z /= rings.size();
        // prev_z = box_B_z;

        // // 生成四边形，保存到点云group中
        // float dis_l = board_l / std::sqrt(2);
        // float step = 0.01f; // 0.5cm
        // float point_start = 0.0f;
        // float point_end = dis_l;
        // PointCloudPtr group(new PointCloud);
        // for (float y = point_start; y <= point_end; y += step)
        // {
        //     PointType pt1, pt2, pt3, pt4;
        //     pt1.x = prev_x;
        //     pt2.x = prev_x;
        //     pt3.x = prev_x;
        //     pt4.x = prev_x;

        //     pt1.y = y + prev_y - dis_l;
        //     pt2.y = y + prev_y;
        //     pt3.y = prev_y + dis_l - y;
        //     pt4.y = prev_y - y;

        //     pt1.z = prev_z + y;
        //     pt2.z = prev_z + dis_l - y;
        //     pt3.z = prev_z - y;
        //     pt4.z = prev_z - dis_l + y;

        //     pt1.intensity = 255.0f;
        //     pt2.intensity = 255.0f;
        //     pt3.intensity = 255.0f;
        //     pt4.intensity = 255.0f;

        //     group->points.push_back(pt1);
        //     group->points.push_back(pt2);
        //     group->points.push_back(pt3);
        //     group->points.push_back(pt4);
        // }
        // //
        // PointCloudPtr ks_cloud1(new PointCloud);
        // PointCloudPtr ks_cloud2(new PointCloud);
        // PointCloudPtr transformed_group2(new PointCloud);
        // Eigen::Vector3d trans1(-prev_x, -prev_y, -prev_z);
        // Eigen::Quaterniond quate1 = eulor_deg_to_q(0.0, 0.0, 0.0);
        // Eigen::Matrix4f mat1 = convertToEigenMatrix4f(trans1, quate1);
        // pcl::transformPointCloud(*group, *ks_cloud1, mat1);

        // Eigen::Vector3d trans2(0, 0, 0);
        // Eigen::Quaterniond quate2 = eulor_deg_to_q(ndt_initial_roll, ndt_initial_pitch, ndt_initial_yaw);
        // Eigen::Matrix4f mat2 = convertToEigenMatrix4f(trans2, quate2);
        // pcl::transformPointCloud(*ks_cloud1, *ks_cloud2, mat2);

        // Eigen::Vector3d trans3(prev_x, prev_y, prev_z);
        // Eigen::Quaterniond quate3 = eulor_deg_to_q(0.0, 0.0, 0.0);
        // Eigen::Matrix4f mat3 = convertToEigenMatrix4f(trans3, quate3);

        // pcl::transformPointCloud(*ks_cloud2, *transformed_group2, mat3);

        // publishPointCloudMsg(boundary_raw_pub, msg->header, transformed_group2);

        // // 生成的四边形与边界点进行ndt+icp配准
        // if (!edge_cloud->empty() && !group->empty())
        // {

        //     pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        //     ndt.setTransformationEpsilon(0.01);
        //     ndt.setResolution(0.1f);
        //     ndt.setMaximumIterations(50);
        //     ndt.setInputSource(edge_cloud);
        //     ndt.setInputTarget(transformed_group2);
        //     Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        //     PointCloudPtr aligned_cloud(new PointCloud);
        //     ndt.align(*aligned_cloud, initial_guess);

        //     // icp配准
        //     pcl::IterativeClosestPoint<PointType, PointType> icp;
        //     icp.setMaximumIterations(200);                               // 迭代次数
        //     icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance); // 最大对应点距离
        //     icp.setTransformationEpsilon(1e-6);                          // 收敛阈值
        //     icp.setEuclideanFitnessEpsilon(1e-6);                        // 欧氏距离阈值
        //     icp.setRANSACIterations(1000);                               // 不使用RANSAC
        //     icp.setInputSource(edge_cloud);
        //     icp.setInputTarget(transformed_group2);
        //     PointCloudPtr icp_aligned_cloud(new PointCloud);
        //     icp.align(*icp_aligned_cloud, ndt.getFinalTransformation());
        //     // printf("ICP score: %f\n", icp.getFitnessScore());
        //     Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
        //     if (icp.getFitnessScore() <= 0.001)
        //     {
        //         PointCloudPtr transformed_group(new PointCloud);
        //         pcl::transformPointCloud(*transformed_group2, *transformed_group, final_transformation);
        //         *group = *transformed_group;
        //     }
        //     else
        //     {
        //         printf("ICP score: %f\n", icp.getFitnessScore());
        //         ROS_WARN("ICP score exceeds threshold, calibration failed.");
        //     }

        //     // 计算变换后的中心点 centroid_target，并添加到group
        //     float sum_x = 0.0f;
        //     float sum_y = 0.0f;
        //     float sum_z = 0.0f;
        //     for (const auto &pt : group->points)
        //     {
        //         sum_x += pt.x;
        //         sum_y += pt.y;
        //         sum_z += pt.z;
        //     }
        //     size_t num_points = group->points.size();
        //     centroid_target.x = sum_x / num_points;
        //     centroid_target.y = sum_y / num_points;
        //     centroid_target.z = sum_z / num_points;
        //     centroid_target.intensity = 10.0f;
        //     group->points.push_back(centroid_target);
        // }

        // 发布生成的四边形 group
        publishPointCloudMsg(boundary_pub, msg->header, group);

        // 发布边缘点
        publishPointCloudMsg(edge_pub, msg->header, edge_cloud);

        // 发布过滤后的点云
        publishPointCloudMsg(target_filtered_pub, msg->header, target_current_cloud);

        if (!target_center_cloud->points.empty())
        {
            publishPointCloudMsg(all_centroid_B_pub, msg->header, target_center_cloud);
        }
        vector<double> marker_B_rgb = {0.0, 1.0, 0.0};
        publishCubeMarker(pub_mark_B, target_frame, box_B_lx, box_B_ly, box_B_lz, box_B_x, box_B_y, box_B_z, box_B_roll, box_B_pitch, box_B_yaw, marker_B_rgb);
    }

    // 给定初始位姿(废弃)
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
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
        // Computer_calib();
    }

    // Flag订阅
    void FlagCallback(const std_msgs::HeaderConstPtr &msg)
    {
        if (msg->frame_id == "calibration_start") // 清空中心点
        {
            ROS_INFO("Calibration started");
            source_center_cloud->points.clear();
            target_center_cloud->points.clear();
            publishflag(0);
        }
        else if (msg->frame_id == "calibration_record_points") // 记录中心点
        {
            source_center_cloud->points.push_back(centroid_source);
            target_center_cloud->points.push_back(centroid_target);
            // printf("Number of recorded points: %ld\n", target_center_cloud->points.size());

            // //  test
            // PointType pt1a, pt1b, pt2a, pt2b, pt3a, pt3b, pt4a, pt4b, pt5a, pt5b;
            // pt1a.x = 1.1;
            // pt1a.y = 2.7;
            // pt1a.z = 5.9;
            // pt1a.intensity = 10.0;
            // pt1b.x = -1.96905;
            // pt1b.y = 3.0694702;
            // pt1b.z = 4.32063;
            // pt1b.intensity = 200.0;
            // source_center_cloud->points.push_back(pt1a);
            // target_center_cloud->points.push_back(pt1b);

            // pt2a.x = 3.4;
            // pt2a.y = 1.0;
            // pt2a.z = 0.2;
            // pt2a.intensity = 10.0;
            // pt2b.x = 2.9981243;
            // pt2b.y = 0.48969842;
            // pt2b.z = 1.26417786;
            // pt2b.intensity = 200.0;
            // source_center_cloud->points.push_back(pt2a);
            // target_center_cloud->points.push_back(pt2b);

            // pt3a.x = 7.9;
            // pt3a.y = 3.8;
            // pt3a.z = 2.1;
            // pt3a.intensity = 10.0;
            // pt3b.x = 5.8514683;
            // pt3b.y = 3.625053;
            // pt3b.z = 4.9693063;
            // pt3b.intensity = 200.0;
            // source_center_cloud->points.push_back(pt3a);
            // target_center_cloud->points.push_back(pt3b);

            // pt4a.x = 4.3;
            // pt4a.y = 6.7;
            // pt4a.z = 9.3;
            // pt4a.intensity = 10.0;
            // pt4b.x = -0.93137337;
            // pt4b.y = 7.6165165;
            // pt4b.z = 8.326579;
            // pt4b.intensity = 200.0;
            // source_center_cloud->points.push_back(pt4a);
            // target_center_cloud->points.push_back(pt4b);

            // pt5a.x = 0.4;
            // pt5a.y = 5.6;
            // pt5a.z = 3.2;
            // pt5a.intensity = 10.0;
            // pt5b.x = -0.780418226;
            // pt5b.y = 5.4775840;
            // pt5b.z = 1.324280886;
            // pt5b.intensity = 200.0;
            // source_center_cloud->points.push_back(pt5a);
            // target_center_cloud->points.push_back(pt5b);
            int cloud_num = source_center_cloud->points.size();
            publishflag(cloud_num);
        }
        else if (msg->frame_id == "calibration_computer") // 进行计算
        {
            estimateRigidTransformationAndPublish(source_center_cloud, target_center_cloud);
        }
        else if (msg->frame_id == "request_erase")
        {
            int this_seq = msg->seq;
            if (this_seq >= 0 && this_seq < source_center_cloud->points.size())
            {
                source_center_cloud->points.erase(source_center_cloud->points.begin() + this_seq);
                source_center_cloud->width = source_center_cloud->points.size();
                source_center_cloud->height = 1;
                source_center_cloud->is_dense = true;
                target_center_cloud->points.erase(target_center_cloud->points.begin() + this_seq);
                target_center_cloud->width = target_center_cloud->points.size();
                target_center_cloud->height = 1;
                target_center_cloud->is_dense = true;
                int cloud_num = source_center_cloud->points.size();
                publishflag(cloud_num);
            }
        }
        else if (msg->frame_id == "auto_find_center")
        {
            auto_find_center_source = true;
            auto_find_center_target = true;
        }
    }

    // XYZIRT点云转XYZI
    PointType convertVelodyneToXYZI(const PointXYZIRT &pt)
    {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        return p;
    }

    PointType convertVelodyneToXYZI(const PointXYZIRT &pt, const int &inten)
    {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = inten;
        return p;
    }

    void insertInterpolatnedPoints(const PointType& p1, const PointType& p2, int n, PointCloudPtr group)
    {
        if (n <= 0) return;  // 没有插值点，直接返回

        for (int i = 1; i <= n; ++i)
        {
            float t = static_cast<float>(i) / n; // t从0到1之间均匀分布，不包含0,包含1

            PointType interp_point;
            interp_point.x = p1.x + t * (p2.x - p1.x);
            interp_point.y = p1.y + t * (p2.y - p1.y);
            interp_point.z = p1.z + t * (p2.z - p1.z);

            group->points.push_back(interp_point);
        }
    }

    // 计算两个点的欧式距离
    double computeEuclideanDistance(const PointType &point1, const PointType &point2)
    {
        return std::sqrt(
            std::pow(point1.x - point2.x, 2) +
            std::pow(point1.y - point2.y, 2) +
            std::pow(point1.z - point2.z, 2));
    }

    double computeEuclideanDistance(const VelodynePointXYZIRT &point1, const VelodynePointXYZIRT &point2)
    {
        return std::sqrt(
            std::pow(point1.x - point2.x, 2) +
            std::pow(point1.y - point2.y, 2) +
            std::pow(point1.z - point2.z, 2));
    }

    PointType computeClosestPointsMidpoint(
        const Eigen::VectorXf &line1_coeffs,
        const Eigen::VectorXf &line2_coeffs)
    {
        // 提取直线1的参数 (p1, d1)
        Eigen::Vector3f p1 = line1_coeffs.head<3>();
        Eigen::Vector3f d1 = line1_coeffs.tail<3>().normalized();

        // 提取直线2的参数 (p2, d2)
        Eigen::Vector3f p2 = line2_coeffs.head<3>();
        Eigen::Vector3f d2 = line2_coeffs.tail<3>().normalized();

        // 计算叉积 n = d1 × d2
        Eigen::Vector3f n = d1.cross(d2);
        float n_norm = n.squaredNorm();

        // 如果两直线平行（n ≈ 0），直接返回 p1（或自定义处理）
        if (n_norm < 1e-6)
        {
            PointType midpoint;
            midpoint.x = (p1.x() + p2.x()) / 2;
            midpoint.y = (p1.y() + p2.y()) / 2;
            midpoint.z = (p1.z() + p2.z()) / 2;
            return midpoint;
        }

        // 计算连接向量 w = p2 - p1
        Eigen::Vector3f w = p2 - p1;

        // 计算 t 和 s
        float t = w.cross(d2).dot(n) / n_norm;
        float s = w.cross(d1).dot(n) / n_norm;

        // 计算最近点 q1 和 q2
        Eigen::Vector3f q1 = p1 + t * d1;
        Eigen::Vector3f q2 = p2 + s * d2;

        // 返回中点
        PointType midpoint;
        midpoint.x = (q1.x() + q2.x()) / 2;
        midpoint.y = (q1.y() + q2.y()) / 2;
        midpoint.z = (q1.z() + q2.z()) / 2;
        return midpoint;
    }

    PointType fitLineAndGetIntersection(
        const PointCloudPtr &cloud1,
        const PointCloudPtr &cloud2)
    {
        // 拟合第一条直线
        pcl::SampleConsensusModelLine<PointType>::Ptr model1(new pcl::SampleConsensusModelLine<PointType>(cloud1));
        pcl::RandomSampleConsensus<PointType> ransac1(model1);
        ransac1.setDistanceThreshold(0.01);
        ransac1.computeModel();
        Eigen::VectorXf coeffs1;
        ransac1.getModelCoefficients(coeffs1); // [px, py, pz, dx, dy, dz]

        // 拟合第二条直线
        pcl::SampleConsensusModelLine<PointType>::Ptr model2(new pcl::SampleConsensusModelLine<PointType>(cloud2));
        pcl::RandomSampleConsensus<PointType> ransac2(model2);
        ransac2.setDistanceThreshold(0.01);
        ransac2.computeModel();
        Eigen::VectorXf coeffs2;
        ransac2.getModelCoefficients(coeffs2);

        return computeClosestPointsMidpoint(coeffs1, coeffs2);

        // // 尝试计算交点
        // PointType intersection;
        // if (pcl::lineToLineSegment(coeffs1, coeffs2, intersection))
        // {
        //     return intersection; // 如果相交，返回交点
        // }
        // else
        // {
        //     // 如果不相交，计算最近点的中点
        //     return computeClosestPointsMidpoint(coeffs1, coeffs2);
        // }
    }

    // 计算两组点云之间的RMSE
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

    // 数组转为变换矩阵4f
    Eigen::Matrix4f convertToEigenMatrix4f(const double trans[7])
    {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::Vector3d translation(trans[0], trans[1], trans[2]);
        Eigen::Quaterniond quaternion(trans[6], trans[3], trans[4], trans[5]);
        Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
        mat.block<3, 3>(0, 0) = rotation;
        mat.block<3, 1>(0, 3) = translation;
        return mat.cast<float>();
    }

    // 转为变换矩阵，输入平移和四元数
    Eigen::Matrix4f convertToEigenMatrix4f(Eigen::Vector3d translation, Eigen::Quaterniond quaternion)
    {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
        mat.block<3, 3>(0, 0) = rotation;
        mat.block<3, 1>(0, 3) = translation;
        return mat.cast<float>();
    }

    // 计算增量变换
    void add_pose()
    {
        double add_trans[7];
        double roll = add_roll * M_PI / 180.0;
        double pitch = add_pitch * M_PI / 180.0;
        double yaw = add_yaw * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf2::Transform add_mat;
        add_mat.setOrigin(tf2::Vector3(add_x, add_y, add_z));
        add_mat.setRotation(q);
        tf2::Transform current_mat;
        current_mat.setOrigin(tf2::Vector3(transformLidarToLidar[0], transformLidarToLidar[1], transformLidarToLidar[2]));
        current_mat.setRotation(tf2::Quaternion(transformLidarToLidar[3], transformLidarToLidar[4], transformLidarToLidar[5], transformLidarToLidar[6]));
        tf2::Transform new_matrix = current_mat * add_mat;

        transformLidarToLidar[0] = new_matrix.getOrigin().x();
        transformLidarToLidar[1] = new_matrix.getOrigin().y();
        transformLidarToLidar[2] = new_matrix.getOrigin().z();
        transformLidarToLidar[3] = new_matrix.getRotation().x();
        transformLidarToLidar[4] = new_matrix.getRotation().y();
        transformLidarToLidar[5] = new_matrix.getRotation().z();
        transformLidarToLidar[6] = new_matrix.getRotation().w();
    }

    // 计算刚体变换
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
        if (transformed_rmse > svd_threshold)
        {
            std::cout << "Transformation does not meet threshold!" << std::endl;
            return;
        }
        std::cout << std::fixed << std::setprecision(9);
        std::cout << std::string(20, '=') << std::endl;
        Eigen::Matrix4f transformation_matrix_inv = transformation_matrix.inverse();
        Eigen::Matrix3d Rtrans = transformation_matrix_inv.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond q_trans(Rtrans);
        Eigen::Vector3d t_trans = transformation_matrix_inv.block<3, 1>(0, 3).cast<double>();
        std::cout << "Transformation results source->target = " << std::endl;
        std::cout << "        t = [";
        std::cout << t_trans[0] << ", " << t_trans[1] << ", " << t_trans[2] << "]" << std::endl;
        std::cout << "        q = [";
        std::cout << q_trans.x() << ", " << q_trans.y() << ", " << q_trans.z() << ", " << q_trans.w() << "]" << std::endl;
        std::cout << std::string(20, '-') << std::endl;

        Eigen::Matrix3d Rtrans2 = transformation_matrix.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond q_trans2(Rtrans2);
        Eigen::Vector3d t_trans2 = transformation_matrix.block<3, 1>(0, 3).cast<double>();
        std::cout << "Transformation results target->source = " << std::endl;
        std::cout << "        t = [";
        std::cout << t_trans2[0] << ", " << t_trans2[1] << ", " << t_trans2[2] << "]" << std::endl;
        std::cout << "        q = [";
        std::cout << q_trans2.x() << ", " << q_trans2.y() << ", " << q_trans2.z() << ", " << q_trans2.w() << "]" << std::endl;
        std::cout << std::string(20, '=') << std::endl;

        transformLidarToLidar[0] = t_trans[0];
        transformLidarToLidar[1] = t_trans[1];
        transformLidarToLidar[2] = t_trans[2];
        transformLidarToLidar[3] = q_trans.x();
        transformLidarToLidar[4] = q_trans.y();
        transformLidarToLidar[5] = q_trans.z();
        transformLidarToLidar[6] = q_trans.w();
    }

    // 点云融合函数
    PointCloudPtr mergePointClouds(const deque<PointCloudPtr> &queue)
    {
        PointCloudPtr merged_cloud(new PointCloud);
        for (const auto &cloud : queue)
        {
            *merged_cloud += *cloud; // Merge point clouds
        }
        return merged_cloud;
    }

    // 重载点云融合函数
    PointXYZIRTCloudPtr mergePointClouds(const deque<PointXYZIRTCloudPtr> &queue)
    {
        PointXYZIRTCloudPtr merged_cloud(new PointXYZIRTCloud);
        for (const auto &cloud : queue)
        {
            *merged_cloud += *cloud; // Merge point clouds
        }
        return merged_cloud;
    }

    // Eigen 欧拉角(角度值)转四元数
    Eigen::Quaterniond eulor_deg_to_q(double roll_deg, double pitch_deg, double yaw_deg)
    {
        double roll = roll_deg * M_PI / 180.0;
        double pitch = pitch_deg * M_PI / 180.0;
        double yaw = yaw_deg * M_PI / 180.0;
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        return q;
    }

    // 发布flag
    void publishflag(const int &this_seq)
    {
        std_msgs::Header header_msg;
        header_msg.stamp = ros::Time::now();
        header_msg.frame_id = "feedback_number";
        header_msg.seq = this_seq;

        feedback_flag_pub.publish(header_msg);
    }

    // 发布点云的函数
    void publishPointCloudMsg(const ros::Publisher &publisher,
                              const std_msgs::Header &header,
                              const PointCloudPtr output_cloud)
    {
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header = header;
        output_msg.header.stamp = ros::Time::now();
        publisher.publish(output_msg);
    }

    // 重载发布点云的函数
    void publishPointCloudMsg(const ros::Publisher &publisher,
                              const std_msgs::Header &header,
                              const PointXYZIRTCloudPtr output_cloud)
    {
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header = header;
        output_msg.header.stamp = ros::Time::now();
        publisher.publish(output_msg);
    }

    // 箱体发布函数
    void publishCubeMarker(ros::Publisher &marker_pub, string frame_id,
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
            {float(lx / 2), float(ly / 2), float(lz / 2)},
            {float(-lx / 2), float(ly / 2), float(lz / 2)},
            {float(-lx / 2), float(-ly / 2), float(lz / 2)},
            {float(lx / 2), float(-ly / 2), float(lz / 2)},
            {float(lx / 2), float(ly / 2), float(-lz / 2)},
            {float(-lx / 2), float(ly / 2), float(-lz / 2)},
            {float(-lx / 2), float(-ly / 2), float(-lz / 2)},
            {float(lx / 2), float(-ly / 2), float(-lz / 2)}};
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
            0, 1, 1, 2, 2, 3, 3, 0, // 前面4条边
            4, 5, 5, 6, 6, 7, 7, 4, // 后面4条边
            0, 4, 1, 5, 2, 6, 3, 7  // 连接前后4条边
        };
        for (int i = 0; i < 24; i += 2)
        {
            marker.points.push_back(vertices_world[edge_indices[i]]);
            marker.points.push_back(vertices_world[edge_indices[i + 1]]);
        }
        marker_pub.publish(marker);
    }

    // 更新param参数
    void updateParam()
    {
        lidar_to_lidar_calibration::TestConfig config;
        config.calib_x = transformLidarToLidar[0];
        config.calib_y = transformLidarToLidar[1];
        config.calib_z = transformLidarToLidar[2];
        Eigen::Quaterniond quaternion_result(transformLidarToLidar[6], transformLidarToLidar[3], transformLidarToLidar[4], transformLidarToLidar[5]);
        Eigen::Vector3d euler_result = quaternion_result.toRotationMatrix().eulerAngles(2, 1, 0); // Yaw, Pitch, Roll
        config.calib_yaw = euler_result[0] * 180.0 / M_PI;                                        // Convert to degrees
        config.calib_pitch = euler_result[1] * 180.0 / M_PI;                                      // Convert to degrees
        config.calib_roll = euler_result[2] * 180.0 / M_PI;                                       // Convert to degrees
        trigger_calculation = false;                                                              // Reset the trigger for next calculation
        config.trigger_calculation = false;                                                       // Reset the trigger for next calculation
        config.icp_score_threshold = icp_score_threshold;
        config.svd_threshold = svd_threshold;

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
        config.ndt_initial_roll = ndt_initial_roll;
        config.ndt_initial_pitch = ndt_initial_pitch;
        config.ndt_initial_yaw = ndt_initial_yaw;
        config.MaxCorrespondenceDistance = MaxCorrespondenceDistance;

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

    // 动参回调函数
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
        svd_threshold = config.svd_threshold;
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
        ndt_initial_roll = config.ndt_initial_roll;
        ndt_initial_pitch = config.ndt_initial_pitch;
        ndt_initial_yaw = config.ndt_initial_yaw;
        board_l = config.board_l;
        MaxCorrespondenceDistance = config.MaxCorrespondenceDistance;

        update_transformLidarToLidar();
        if (add_x != 0.0 || add_y != 0.0 || add_z != 0.0 ||
            add_roll != 0.0 || add_pitch != 0.0 || add_yaw != 0.0)
        {
            add_pose();
            update_pose = true;
        }
        ROS_INFO("Dynamic reconfigure parameters updated!");
    }

    // 独立线程，发布 tf
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
                if (tf_publish)
                {
                    tf_broadcaster.sendTransform(transformStamped);
                }
                // vector<double> marker_A_rgb = {1.0, 0.0, 0.0};

                // publishCubeMarker(pub_mark_A, source_frame, box_A_lx, box_A_ly, box_A_lz, box_A_x, box_A_y, box_A_z, box_A_roll, box_A_pitch, box_A_yaw, marker_A_rgb);
                // vector<double> marker_B_rgb = {0.0, 1.0, 0.0};
                // publishCubeMarker(pub_mark_B, target_frame, box_B_lx, box_B_ly, box_B_lz, box_B_x, box_B_y, box_B_z, box_B_roll, box_B_pitch, box_B_yaw, marker_B_rgb);
            }
            if (update_pose)
            {
                ros::Duration duration(0.1);
                updateParam();
            }

            if (trigger_calculation)
            {
                ROS_INFO("Triggering calibration calculation.");
                // Computer_calib();
            }
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");

    LidarCalibration LC;

    ROS_INFO("\033[1;32m%s\033[0m", "Lidar Calibration Node Started");

    std::thread publishtfthread(&LidarCalibration::publishTF, &LC);

    ros::spin();
    publishtfthread.join();
    return 0;
}