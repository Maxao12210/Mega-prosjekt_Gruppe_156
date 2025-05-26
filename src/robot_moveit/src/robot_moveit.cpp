#include <memory>
#include "staticPositions.cpp"
#include "Static_pos.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <object_reference_msg/srv/object_reference.hpp>

// Source link: https://moveit.picknik.ai/main/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html

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

// Initiate global ref_pos values for the sevice
double target_x, target_y;
bool response_check;

// Get tcp pos values from get_tcp_pos service
void get_tcp_pos(const std::shared_ptr<object_reference_msg::srv::ObjectReference::Request> request,
                 std::shared_ptr<object_reference_msg::srv::ObjectReference::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f",
                request->x, request->y);
  if (0.1 <= request->x && request->x <= 1.00 && 0.1 <= request->y  && request->y <= 0.5) // Usikre verdier
  {
    target_x = request->x;
    target_y = request->y;
    response->success = true;
  } else {
    response->success = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request failed!");
  }
  response_check = response->success;
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
  plan_and_execute(move_group, coordinatesForHome, logger, "home position");

  // MOve robot to first ref point
  plan_and_execute(move_group, ref_joint_values, logger, "reference position");

  bool running = true;

  // Sjekke om service responsen har fått gyldige verdier, og setter ny posisjon til robot og planer
  // NB! Forikre at vardian e bra før execute, har ikke testet selv
  while(running) {

    if (response_check) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New planning accepted");
      // Set a target Pose
      auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = target_x;
        msg.position.y = target_y;
        msg.position.z = 0.5;
        return msg;
      }();
      move_group.setPoseTarget(target_pose);

      // Create a plan to that target pose
      auto const [success, plan] = [&move_group]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
      }();

      // Execute the plan, uncoment to execute
      //if(success) {
      //move_group.execute(plan);
      //} else {
      //RCLCPP_ERROR(logger, "Planning failed!");
      //}
      std::string terminal_response;
      std::cin >> terminal_response;
      if (terminal_response == "[INFO] [1748276711.937318770] [moveit_3700910046.moveit.ros.move_group_interface]: Execute request success!") {
        running = false;
        response_check = false;
      }
    }


  }

  rclcpp::spin(node); // Ensure the node stays alive for get_tcp_pos service to work
  rclcpp::shutdown();
  return 0;
}

