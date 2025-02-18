#ifndef KEYBOARD_PUBLISHER_H
#define KEYBOARD_PUBLISHER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "interfaces/msg/arm_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

bool isOpen = true;

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher();

 private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
  size_t count_;
  double defSpeed = 10;
};

#endif
