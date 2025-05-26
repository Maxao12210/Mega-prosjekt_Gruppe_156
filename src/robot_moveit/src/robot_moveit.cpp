#include <memory>
#include "staticPositions.h"
#include "referencePosition.hpp"
#include "PathPlanning.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "object_reference_msg/srv/object_reference.hpp"


// Source link: https://moveit.picknik.ai/main/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html

PathPlanning plan_and_execute;

staticPositions homePosition("Home position", 0.000,-1.571,-0.000,0.000,0.000,0.000);
std::map<std::string, double> coordinatesForHome = homePosition.getCoordinatesMap();

staticPositions ref1("Position nr.1", 0.005, -1.060, 0.3875, -0.930, -1.474,-0.0005);
std::map<std::string, double> coordinatesForRef1 = ref1.getCoordinatesMap();

// Not accurate values
double orientation_x = 0.7, orientation_y = 0.7, orientation_z = 0.7, orientation_w = 0.7;
double target_x1, target_y1, target_x2, target_y2, target_x3, target_y3;
double position_z = 0.3;

bool response_check = false;

// Get tcp pos values from get_tcp_pos service
void get_tcp_pos(const std::shared_ptr<object_reference_msg::srv::ObjectReference::Request> &request,
                 const std::shared_ptr<object_reference_msg::srv::ObjectReference::Response> &response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx1: %f" " y1: %f" "x2: %f" "y2: %f" "x3: %f" "y3: %f",
                  request->x1, request->y1, request->x2, request->y2, request->x3, request->y3);

    if ((0.2 <= request->x1 && request->x1 <= 0.5) &&
        (0.2 <= request->x2 && request->x2 <= 0.5) &&
        (0.2 <= request->x3 && request->x3 <= 0.5) &&
        (0.1 <= request->y1 && request->y1 <= 0.4) &&
        (0.1 <= request->y2 && request->y2 <= 0.4) &&
        (0.1 <= request->y3 && request->y3 <= 0.4)) // Usikre verdier
    {
        target_x1 = request->x1;
        target_y1 = request->y1;
        target_x2 = request->x2;
        target_y2 = request->y2;
        target_x3 = request->x3;
        target_y3 = request->y3;
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
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForHome, logger, "home position");

  // MOve robot to first ref point
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  // Spin/wait for callback to set the flag
  while (!response_check) {
    rclcpp::spin_some(node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for ref pos");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // avoid busy loop
  }

referencePosition firstPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x1, target_y1, position_z);
geometry_msgs::msg::Pose firstPose = firstPosition.getPose();

referencePosition secondPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x2, target_y2, position_z);
geometry_msgs::msg::Pose secondPose = secondPosition.getPose();

referencePosition thirdPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x3, target_y3, position_z);
geometry_msgs::msg::Pose thirdPose = thirdPosition.getPose();

  // Move above Object positions
  plan_and_execute.plan_and_execute_tcp(move_group, firstPose, logger, "target position1");

  plan_and_execute.plan_and_execute_tcp(move_group, secondPose, logger, "target position2");

  plan_and_execute.plan_and_execute_tcp(move_group, thirdPose, logger, "target position3");

  // MOve robot to first ref point
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  rclcpp::shutdown();
  return 0;
}

