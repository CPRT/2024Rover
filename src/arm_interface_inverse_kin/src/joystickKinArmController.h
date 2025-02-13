#ifndef JOYSTICKKINARMCONTROLLER_H
#define JOYSTICKKINARMCONTROLLER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/arm_cmd.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoystickKinPublisher : public rclcpp::Node
{
public:
    JoystickKinPublisher();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    double defSpeed = 10.0;
};


#endif