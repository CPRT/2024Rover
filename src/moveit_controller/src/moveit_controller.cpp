#include "moveit_controller.h"

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;

void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj,
                       moveit::planning_interface::MoveGroupInterfacePtr mgi) {
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Starting thread");
  mgi->asyncExecute(traj);
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Ending thread");
}

void executePlan(
    moveit::planning_interface::MoveGroupInterface::Plan &rotationPlan,
    moveit::planning_interface::MoveGroupInterfacePtr mgi) {
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Starting thread");
  mgi->asyncExecute(rotationPlan);
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Ending thread");
}

bool isEmpty(const geometry_msgs::msg::Pose &p) {
  return p.position.x == 0 && p.position.y == 0 && p.position.z == 0 &&
         p.orientation.x == 0 && p.orientation.y == 0 && p.orientation.z == 0 &&
         p.orientation.w == 0;
}

MoveitController::MoveitController(const rclcpp::NodeOptions &options)
    : Node("hello_moveit", options),
      node_ptr(std::make_shared<rclcpp::Node>("example_moveit")),
      executor_ptr(
          std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
  subscription = this->create_subscription<interfaces::msg::ArmCmd>(
      "arm_base_commands", 10,
      std::bind(&MoveitController::topic_callback, this,
                std::placeholders::_1));

  move_group_ptr =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node_ptr, "rover_arm3");  // used to be rover_arm, then rover_arm2

  executor_ptr->add_node(node_ptr);
  executor_thread = std::thread([this]() { this->executor_ptr->spin(); });

  // default pose, chosen randomly
  default_pose.position.x = ARM_DEFAULT_X;
  default_pose.position.y = ARM_DEFAULT_Y;
  default_pose.position.z = ARM_DEFAULT_Z;

  geometry_msgs::msg::Pose target_pose;
  target_pose.position = default_pose.position;
  move_group_ptr->setMaxVelocityScalingFactor(1.0);
  move_group_ptr->setMaxAccelerationScalingFactor(1.0);
  move_group_ptr->setPoseTarget(target_pose);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_ptr->plan(plan));

  // Execute the plan
  if (success) {
    move_group_ptr->execute(plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planing failed!");
  }
}

void MoveitController::topic_callback(const interfaces::msg::ArmCmd &armMsg) {
  geometry_msgs::msg::Pose poseMsg = armMsg.pose;
  double stepSize = armMsg.speed;
  RCLCPP_INFO(this->get_logger(), "I heard: %f %f %f %f %f %f %f",
              poseMsg.position.x, poseMsg.position.y, poseMsg.position.z,
              poseMsg.orientation.x, poseMsg.orientation.y,
              poseMsg.orientation.z,
              poseMsg.orientation.w);  //*/

  move_group_ptr->stop();
  if (move_it_thread.joinable()) {
    move_it_thread.join();
  }
  if ((isEmpty(poseMsg) && !armMsg.reset &&
       armMsg.end_effector == interfaces::msg::ArmCmd::END_EFF_UNKNOWN &&
       !armMsg.query_goal_state) ||
      armMsg.estop) {
    RCLCPP_INFO(this->get_logger(), "Stopping the arm");
    return;
  }

  std::vector<geometry_msgs::msg::Pose> points;

  move_group_ptr->setStartStateToCurrentState();
  geometry_msgs::msg::Pose current_pose = move_group_ptr->getCurrentPose().pose;

  points.push_back(current_pose);

  // Print the current pose of the end effector
  RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
              current_pose.position.x, current_pose.position.y,
              current_pose.position.z, current_pose.orientation.x,
              current_pose.orientation.y, current_pose.orientation.z,
              current_pose.orientation.w);  //*/

  if (armMsg.end_effector != interfaces::msg::ArmCmd::END_EFF_UNKNOWN) {
    // TODO: Add end-effector features
  }
  if (armMsg.reset == true)  // reset to default position
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = default_pose.position;
    move_group_ptr->setPoseTarget(target_pose);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_ptr->plan(plan));

    // Execute the plan
    if (success) {
      RCLCPP_INFO(
          this->get_logger(),
          "Number of joint trajectory points: %li, number of "
          "multiDOFjointtrajectorypoints: %li",
          std::size(plan.trajectory_.joint_trajectory.points),
          std::size(plan.trajectory_.multi_dof_joint_trajectory.points));
      move_group_ptr->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }
  } else {
    geometry_msgs::msg::Pose new_pose = current_pose;
    new_pose.position.x += poseMsg.position.x * stepSize;
    new_pose.position.y += poseMsg.position.y * stepSize;
    new_pose.position.z += poseMsg.position.z * stepSize;

    if (poseMsg.orientation.x != 0 || poseMsg.orientation.y != 0 ||
        poseMsg.orientation.z != 0 ||
        poseMsg.orientation.w != 0)  // rotation required
    {
      tf2::Quaternion q1;
      tf2::convert(current_pose.orientation, q1);
      tf2::Quaternion q2;
      q2.setRPY(poseMsg.orientation.x, poseMsg.orientation.y,
                poseMsg.orientation.z);
      tf2::Quaternion q3 = q1 * q2;
      geometry_msgs::msg::Quaternion q4 = tf2::toMsg(q3);

      new_pose.orientation = q4;
    }
    points.push_back(new_pose);
    move_group_ptr->computeCartesianPath(points, EEF_STEP, JUMP_THRESHOLD,
                                         trajectory);

    // launch thread
    move_it_thread =
        std::thread(executeTrajectory, std::ref(trajectory), move_group_ptr);
    RCLCPP_INFO(this->get_logger(), "Thread created");
  }
}

int main(int argc, char **argv) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveitController>(
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  rclcpp::spin(node);
}
