#pragma once

#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace aloam_velodyne
{

class LaserPGONode : public rclcpp::Node
{
public:
  explicit LaserPGONode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LaserPGONode() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_full_res_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_laser_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  std::vector<std::thread> worker_threads_;
};

}  // namespace aloam_velodyne
