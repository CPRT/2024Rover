#include "joystickKinArmController.h"

using namespace std::chrono_literals;

JoystickKinPublisher::JoystickKinPublisher(): Node("minimal_publisher"){
  publisher_ = this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 10);
  //timer_ = this->create_wall_timer(500ms, std::bind(&JoystickKinPublisher::timer_callback, this));
  subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoystickKinPublisher::joy_callback, this, std::placeholders::_1)
    );
}

void JoystickKinPublisher::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
	//form pose command message
    interfaces::msg::ArmCmd poseCmd = [&]{
		interfaces::msg::ArmCmd msg;
		msg.pose.position.x = 0;
		msg.pose.position.y = 0;
		msg.pose.position.z = 0;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.y = 0;
		msg.pose.orientation.z = 0;
		msg.pose.orientation.w = 0;
		msg.speed = defSpeed; //ask Will how the default is set and how it changes. and like what speed even means in this context.
		// its not speed. its step size, 10 is to make it smooth as it moves.
		msg.named_pose = 0;
		msg.estop = false;
		msg.reset = false;
		msg.query_goal_state = false;
		return msg;
	}();
	//form geo message message for current position
	//dont know what those numbers are for yet... starting position? zero'd? Will didnt know.
	geometry_msgs::msg::Pose current_pose = []{
		geometry_msgs::msg::Pose msg;
		msg.orientation.w = 1.0;
		msg.position.x = 0.636922;
		msg.position.y = 0.064768;
		msg.position.z = 0.678810;
		return msg;
	}();

	//joystick commands
	if (joy_msg->axes[5] > 0.25) {
    poseCmd.pose.position.x = 1;
	} 
	else if (joy_msg->axes[5] < -0.25) {
    poseCmd.pose.position.x = -1;
	} else {poseCmd.pose.position.x = 0;}

	if (joy_msg->axes[4] > 0.25) {
    poseCmd.pose.position.y = 1;
	} 
	else if (joy_msg->axes[4] < -0.25) {
    poseCmd.pose.position.y = -1;
	} else {poseCmd.pose.position.y = 0;}

	//publish commands
	poseCmd.current_pose = current_pose;
  	publisher_->publish(poseCmd);
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickKinPublisher>());
  rclcpp::shutdown();
  return 0;
}