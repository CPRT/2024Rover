#include "ThrustmasterControl.hpp"

ThrustmasterControl::ThrustmasterControl()
    : Node("thrustmaster_control"),
      mode_(nullptr),
      currentMode_(ModeType::NONE) {
  loadParameters();
  changeMode(currentMode_);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ThrustmasterControl::processJoystick, this,
                std::placeholders::_1));
}

void ThrustmasterControl::processJoystick(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (checkForModeChange(joystickMsg)) {
    return;
  }
  if (mode_ != nullptr && currentMode_ != ModeType::NONE) {
    mode_->processJoystickInput(joystickMsg);
  }
}

bool ThrustmasterControl::checkForModeChange(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (joystickMsg->buttons[kDriveModeButton]) {
    return changeMode(ModeType::DRIVE);
  } else if (joystickMsg->buttons[kArmIKModeButton]) {
    return changeMode(ModeType::ARM_IK);
  } else if (joystickMsg->buttons[kArmManualModeButton]) {
    return changeMode(ModeType::ARM_MANUAL);
  } else if (joystickMsg->buttons[kNavModeButton]) {
    return changeMode(ModeType::NAV);
  } else if (joystickMsg->buttons[kScienceModeButton]) {
    return changeMode(ModeType::SCIENCE);
  }
  return false;
}

bool ThrustmasterControl::changeMode(ModeType mode) {
  if (currentMode_ == mode) {
    return false;
  }
  currentMode_ = mode;
  switch (mode) {
    case ModeType::NONE:
      mode_ = nullptr;
      return true;
    case ModeType::DRIVE:
      RCLCPP_INFO(this->get_logger(), "Entering Drive Mode");
      mode_ = std::make_unique<DriveMode>(this);
      return true;
    default:
      RCLCPP_WARN(this->get_logger(),
                  "Mode not implemented, returning to NONE");
      currentMode_ = ModeType::NONE;
      mode_ = nullptr;
      return false;
  }
  return false;
}

void ThrustmasterControl::loadParameters() {
  this->declare_parameter("drive_mode_button", 12);
  this->declare_parameter("arm_ik_mode_button", 11);
  this->declare_parameter("arm_manual_mode_button", 10);
  this->declare_parameter("nav_mode_button", 13);
  this->declare_parameter("science_mode_button", 14);
  this->get_parameter("drive_mode_button", kDriveModeButton);
  this->get_parameter("arm_ik_mode_button", kArmIKModeButton);
  this->get_parameter("arm_manual_mode_button", kArmManualModeButton);
  this->get_parameter("nav_mode_button", kNavModeButton);
  this->get_parameter("science_mode_button", kScienceModeButton);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrustmasterControl>());
  rclcpp::shutdown();
  return 0;
}