#include "TalonDriveController.hpp"

#include <algorithm>

TalonDriveController::TalonDriveController() : Node("talonDrive") {
  lastTimestamp_ = 0;
  this->declare_parameter("max_speed", 2.0);
  maxSpeed_ = this->get_parameter("max_speed").as_double();
  this->declare_parameter("base_width", 0.9144);
  baseWidth_ = this->get_parameter("base_width").as_double();
  this->declare_parameter("pub_odom", true);
  pubOdom_ = this->get_parameter("pub_odom").as_bool();
  this->declare_parameter("pub_elec", true);
  pubElec_ = this->get_parameter("pub_elec").as_bool();
  this->declare_parameter("frequency", 10.0);
  const double frequency =
      std::max(this->get_parameter("frequency").as_double(), 1.0);

  this->declare_parameter(
      "wheels", std::vector<std::string>{"frontRight", "frontLeft", "backRight",
                                         "backLeft"});
  std::vector<std::string> wheel_names =
      this->get_parameter("wheels").as_string_array();

  for (const auto &wheel_name : wheel_names) {
    wheels_.emplace_back(WheelControl(wheel_name, this));
    if (pubOdom_) {
      statusSubs_.emplace_back(this->create_subscription<MotorStatus>(
          wheel_name + "/status", 10,
          std::bind(&WheelControl::setStatus, &wheels_.back(),
                    std::placeholders::_1)));
    }
  }

  RCLCPP_INFO(this->get_logger(), "There are %ld wheels in the drive",
              wheels_.size());

  twistSub_ = this->create_subscription<Twist>(
      "/cmd_vel", 10,
      std::bind(&TalonDriveController::twist_callback, this,
                std::placeholders::_1));
  const int periodMs = 1000 / frequency;
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(periodMs),
      std::bind(&TalonDriveController::control_timer_callback, this));

  if (pubOdom_) {
    odomPub_ = this->create_publisher<Odometry>("/drive/odom", 10);
    odomTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(periodMs),
        std::bind(&TalonDriveController::odom_pub_callback, this));
  }
}

void TalonDriveController::odom_pub_callback() {
  double vl_total = 0.0;
  double vr_total = 0.0;

  for (const auto &wheel : wheels_) {
    if (wheel.getWheelSide() == WheelSide::LEFT) {
      vl_total += wheel.getVelocity();
    } else {
      vr_total += wheel.getVelocity();
    }
  }

  double vl = vl_total / (wheels_.size() / 2);
  double vr = vr_total / (wheels_.size() / 2);

  Odometry odom;
  odom.header.stamp = this->get_clock()->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";  // TODO: make configurable
  odom.twist.twist.linear.x = (vr + vl) / 2;
  odom.twist.twist.angular.z = (vl - vr) / baseWidth_;
  odom.twist.covariance = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3};  // TODO: make configurable
  odomPub_->publish(odom);
}

void TalonDriveController::control_timer_callback() {
  if (this->get_clock()->now().seconds() - lastTimestamp_ > 2) {
    for (auto &wheel : wheels_) {
      wheel.setVelocity(0.0);
    }
  }
  for (const auto &wheel : wheels_) {
    wheel.send();
  }
}

void TalonDriveController::twist_callback(const Twist::SharedPtr msg) {
  lastTimestamp_ = this->get_clock()->now().seconds();

  double linearX = msg->linear.x;
  if (linearX > maxSpeed_) {
    linearX = maxSpeed_;
  }
  if (linearX < -maxSpeed_) {
    linearX = -maxSpeed_;
  }

  double vr = linearX + msg->angular.z * baseWidth_ / 2;
  double vl = linearX - msg->angular.z * baseWidth_ / 2;

  for (auto &wheel : wheels_) {
    if (wheel.getWheelSide() == WheelSide::LEFT) {
      wheel.setVelocity(vl);
    } else {
      wheel.setVelocity(-vr);
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalonDriveController>());
  rclcpp::shutdown();
  return 0;
}
