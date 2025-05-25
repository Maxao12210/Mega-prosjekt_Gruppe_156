#include <memory>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "robot_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("robot_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Create a map of joint names to target values (in radians or meters)
  std::map<std::string, double> home_joint_values;
  home_joint_values["shoulder_lift_joint"] = -1.571; // radians
  home_joint_values["elbow_joint"] = -0.000;
  home_joint_values["wrist_1_joint"] = 0.000;
  home_joint_values["wrist_2_joint"] = 0.000;
  home_joint_values["wrist_3_joint"] = 0.000;
  home_joint_values["shoulder_pan_joint"] = 0.000;

  move_group_interface.setJointValueTarget(home_joint_values);
  
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Create a map of joint names to target values (in radians or meters)
  std::map<std::string, double> refPos;
  refPos["shoulder_lift_joint"] = -1.060; // radians
  refPos["elbow_joint"] = 0.3875;
  refPos["wrist_1_joint"] = -0.930;
  refPos["wrist_2_joint"] = -1.474;
  refPos["wrist_3_joint"] = -0.0005;
  refPos["shoulder_pan_joint"] = 0.005;

  move_group_interface.setJointValueTarget(refPos);

  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success2) {
    move_group_interface.execute(plan2);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
