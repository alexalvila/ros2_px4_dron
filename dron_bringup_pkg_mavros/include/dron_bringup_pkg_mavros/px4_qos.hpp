#pragma once

#include <rclcpp/rclcpp.hpp>

namespace dron_bringup_pkg_mavros
{

inline rclcpp::QoS px4_qos(std::size_t depth = 1)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth))
    .best_effort()
    .transient_local();
}

}  // namespace dron_bringup_pkg_mavros_mavros
