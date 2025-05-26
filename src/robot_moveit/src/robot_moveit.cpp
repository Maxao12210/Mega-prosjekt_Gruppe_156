#include <memory>
#include "staticPositions.cpp"
#include "referencePosition.cpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>


// Source link: https://moveit.picknik.ai/main/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html

// Helper function to plan and execute joint movements
bool plan_and_execute_joint(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::map<std::string, double> & joint_targets,
  const rclcpp::Logger & logger,
  const std::string & description)
{
  move_group.setJointValueTarget(joint_targets);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));
  if (success) {
    RCLCPP_INFO(logger, "Planning to %s successful, executing...", description.c_str());
    move_group.execute(plan);
    return true;
  } else {
    RCLCPP_ERROR(logger, "Planning to %s failed!", description.c_str());
    return false;
  }
}

// Helper function to plan and execute tcp position
bool plan_and_execute_tcp(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target_pos,
  const rclcpp::Logger & logger,
  const std::string & description)
{
  move_group.setPoseTarget(target_pos);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));
  if (success) {
    RCLCPP_INFO(logger, "Planning to %s successful, executing...", description.c_str());
    move_group.execute(plan);
    return true;
  } else {
    RCLCPP_ERROR(logger, "Planning to %s failed!", description.c_str());
    return false;
  }
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "robot_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("robot_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_manipulator");

  // Start get_tcp_pos service
  rclcpp::Service<object_reference_msg::srv::ObjectReference>::SharedPtr service =
   node->create_service<object_reference_msg::srv::ObjectReference>("get_tcp_pos",  &get_tcp_pos);

  // Move robot to home pos
  plan_and_execute_joint(move_group, coordinatesForHome, logger, "home position");

  // MOve robot to first ref point
  plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  // Spin/wait for callback to set the flag
  while (!response_check) {
    rclcpp::spin_some(node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for ref pos");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // avoid busy loop
  }

  // Move above Object positions
  plan_and_execute_tcp(move_group, firstPose, logger, "target position");

  plan_and_execute_tcp(move_group, secondPose, logger, "target position");

  plan_and_execute_tcp(move_group, thirdPose, logger, "target position");

  // MOve robot to first ref point
  plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  rclcpp::shutdown();
  return 0;
}

