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
  std::map<std::string, double> target_joint_values;
  target_joint_values["shoulder_lift_joint"] = -1.571; // radians
  target_joint_values["elbow_joint"] = -0.000;
  target_joint_values["wrist_1_joint"] = 0.000;
  target_joint_values["wrist_2_joint"] = 0.000;
  target_joint_values["wrist_3_joint"] = 0.000;
  target_joint_values["shoulder_pan_joint"] = 0.000;

  move_group_interface.setJointValueTarget(target_joint_values);
  
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

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
