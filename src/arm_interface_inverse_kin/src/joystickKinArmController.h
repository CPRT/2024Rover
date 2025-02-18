#ifndef JOYSTICKKINARMCONTROLLER_H
#define JOYSTICKKINARMCONTROLLER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "interfaces/msg/arm_cmd.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

bool isEqual(interfaces::msg::ArmCmd a, interfaces::msg::ArmCmd b);

class JoystickReader : public rclcpp::Node {
 public:
  JoystickReader();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sol_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr light_publisher_;
  rclcpp::Client<interfaces::srv::MoveServo>::SharedPtr client_;
  interfaces::msg::ArmCmd oldCmd;
  rclcpp::TimerBase::SharedPtr timer_;
  bool shouldPub = false;

  void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publish_message();
  void send_request(int port, int pos, int min, int max);
  void servo_request(int req_port, int req_pos, int req_min, int req_max);
};

int main(int argc, char** argv);

#endif  // JOYSTICKKINARMCONTROLLER_H
