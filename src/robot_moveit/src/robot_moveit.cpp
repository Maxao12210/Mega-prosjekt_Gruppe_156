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

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation. = 0.709;
    msg.orientation.y = -0.705;
    msg.orientation.z = -0.016;
    msg.orientation.w = 0.008;
    msg.position.x = 0.533;
    msg.position.y = 0.115;
    msg.position.z = 0.107;
    return msg;
  }();
  
  //move_group.setPlanningTime(5.0);

  move_group_interface.setPoseTarget(target_pose);
  
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  //if(success) {
    //move_group_interface.execute(plan);
  //} else {
    //RCLCPP_ERROR(logger, "Planning failed!");
  //}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
