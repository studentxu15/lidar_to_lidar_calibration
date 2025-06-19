cmake_minimum_required(VERSION 3.0.2)
project(lidar_to_lidar_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  dynamic_reconfigure
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  config/Test.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs tf2_ros pcl_conversions dynamic_reconfigure
)

# catkin_install_python(PROGRAMS
#   scripts/calib_sync.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/calibration.cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
