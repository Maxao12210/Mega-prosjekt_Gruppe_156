#include <memory>
#include <map>
#include <string>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

// Helper function to plan and execute joint movements
bool plan_and_execute(
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

  // Define "home" joint positions
  std::map<std::string, double> home_joint_values = {
    {"shoulder_pan_joint",    0.000},
    {"shoulder_lift_joint",  -1.571},
    {"elbow_joint",          -0.000},
    {"wrist_1_joint",         0.000},
    {"wrist_2_joint",         0.000},
    {"wrist_3_joint",         0.000}
  };

  plan_and_execute(move_group, home_joint_values, logger, "home position");

  // Define reference joint positions
  std::map<std::string, double> ref_joint_values = {
    {"shoulder_pan_joint",    0.005},
    {"shoulder_lift_joint",  -1.060},
    {"elbow_joint",           0.3875},
    {"wrist_1_joint",        -0.930},
    {"wrist_2_joint",        -1.474},
    {"wrist_3_joint",        -0.0005}
  };

  plan_and_execute(move_group, ref_joint_values, logger, "reference position");

  rclcpp::shutdown();
  return 0;
}
