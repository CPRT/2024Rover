#ifndef ROS_PHOENIX_PHOENIX_COMPONENT
#define ROS_PHOENIX_PHOENIX_COMPONENT

#include "ros_phoenix/phoenix_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ros_phoenix {

template <class PhoenixNodeType>
class PhoenixComponent : public PhoenixNodeType {
 public:
  PhoenixComponent(const NodeOptions& options = NodeOptions())
      : PhoenixNodeType("phoenix_component", options) {
    const std::string name(this->get_name());
    this->pub_ = this->template create_publisher<ros_phoenix::msg::MotorStatus>(
        name + "/status", 1);
    this->sub_ =
        this->template create_subscription<ros_phoenix::msg::MotorControl>(
            name + "/set", 1,
            std::bind(&BaseNode::set, this, std::placeholders::_1));
    this->srv_ = this->template create_service<std_srvs::srv::Trigger>(
        name + "/reset_encoder",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          (void)request;  // Unused parameter
          response->success = this->resetEncoder();
          response->message = response->success ? "true" : "false";
        });
  }

 private:
  virtual void onTimer() {
    BaseNode::onTimer();
    // TODO Improve this
    this->pub_->publish(*(this->status()));
  }

  rclcpp::Publisher<ros_phoenix::msg::MotorStatus>::SharedPtr pub_;
  rclcpp::Subscription<ros_phoenix::msg::MotorControl>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

using TalonFX = PhoenixComponent<TalonFXNode>;
using TalonSRX = PhoenixComponent<TalonSRXNode>;
using VictorSPX = PhoenixComponent<VictorSPXNode>;

}  // namespace ros_phoenix

#endif  // ROS_PHOENIX_PHOENIX_COMPONENT