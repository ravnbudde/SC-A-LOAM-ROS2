#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

inline rclcpp::Time rosTimeFromSec(double seconds)
{
  const auto nanoseconds = static_cast<int64_t>(std::llround(seconds * 1e9));
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

inline double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).seconds();
}

template <typename T>
T declareAndGet(rclcpp::Node * node, const std::string & name, const T & default_value)
{
  node->declare_parameter<T>(name, default_value);
  return node->get_parameter(name).get_value<T>();
}

template <typename T>
T declareAndGet(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  return declareAndGet<T>(node.get(), name, default_value);
}
