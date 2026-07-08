cmake_minimum_required(VERSION 3.16)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

set(ROS_DEPENDENCIES
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(alaserPGO
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
ament_target_dependencies(alaserPGO ${ROS_DEPENDENCIES})
target_compile_options(alaserPGO PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(alaserPGO
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

install(TARGETS alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz_cfg utils
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${ROS_DEPENDENCIES})
ament_package()
