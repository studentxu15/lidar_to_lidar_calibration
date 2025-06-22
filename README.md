# Lidar Calibration Node

## 概述

本节点用于实现激光雷达（LiDAR）之间的标定。通过订阅源激光雷达和目标激光雷达的点云数据，计算两者之间的变换关系，并发布相应的变换结果。节点还支持通过动态参数配置进行标定参数的调整。

## 依赖

- ROS (Robot Operating System)
- `tf2_ros` 用于发布坐标变换
- `dynamic_reconfigure` 用于动态参数配置
- `pcl_ros` 用于点云数据处理

## 编译

1. 确保已经安装了ROS和相关的依赖包。
2. 将本节点放入你的ROS工作空间的`src`目录下。
3. 在工作空间根目录下运行以下命令进行编译：

   ```bash
   catkin_make

## 运行
1. 在工作空间根目录下运行以下命令进行编译：
   ```bash
   source devel/setup.bash
   ```
   ```bash
   roslaunch lidar_to_lidar_calibration lidar_to_lidar_calibration.launch
   ```
2. 确保源激光雷达和目标激光雷达的数据已经发布到相应的ROS话题上。
   
## 参数配置
1. 运行文件参数配置
2. 动态参数配置。
   
    节点支持通过**dynamic_reconfigure**进行动态参数配置。可以通过以下命令启动动态参数配置界面：
    ```bash
    rosrun rqt_reconfigure rqt_reconfigure
    ```
