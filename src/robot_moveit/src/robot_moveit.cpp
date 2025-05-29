#include <algorithm>  // for std::min, std::max
#include <memory>
#include "../include/robot_moveit/StaticPositions.hpp"
#include "robot_moveit/ReferencePosition.hpp"
#include "robot_moveit/PlanningExecute.hpp"
#include "robot_moveit/CameraBracket.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "object_reference_msg/srv/object_reference.hpp"

// Source link: https://moveit.picknik.ai/main/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html

// Create a planning and execute object
PlanningExecute plan_and_execute;

// Create home position with coordinates
StaticPositions homePosition("Home position", 0.000,-1.571,-0.000,0.000,0.000,0.000);
std::map<std::string, double> coordinatesForHome = homePosition.getCoordinatesMap();

// Create first position for picture reference with coordinates
StaticPositions ref1("Position nr.1", 0.005, -1.060, 0.3875, -0.930, -1.57,-0.0005);
std::map<std::string, double> coordinatesForRef1 = ref1.getCoordinatesMap();

// Create second position for picture reference with coordinates
StaticPositions ref2("Position nr.1", 0.005, -0.777, 0.3064, -1.132, -1.57,-0.0005);
std::map<std::string, double> coordinatesForRef2 = ref2.getCoordinatesMap();

// Values to lock the TCP in an orientation while moving to box positions
double orientation_x = 0.709, orientation_y = -0.704, orientation_z = -0.023, orientation_w = -0.045;
double target_x1, target_y1, target_x2, target_y2, target_x3, target_y3;
double position_z = 0.25; // Pointing hight

// Restrictions for tcp x- and y positions to avoid invalid path planning
double min_x = 0.15;
double min_y = -0.13;
double max_x = 0.6;
double max_y = 0.45;

// Global check for acquired box position to robot tcp
bool response_check = false;

// Callback function to get tcp x- and y positions for robot tcp
void get_tcp_pos_callback(
  const std::shared_ptr<object_reference_msg::srv::ObjectReference::Request>  request,
  std::shared_ptr<object_reference_msg::srv::ObjectReference::Response>       response)
{
  auto lg = rclcpp::get_logger("robot_moveit");
  RCLCPP_INFO(
    lg,
    "Incoming raw coords: R:(%f, %f), G:(%f, %f), B:(%f, %f)",
    request->x1, request->y1,
    request->x2, request->y2,
    request->x3, request->y3);

  // clamp each coordinate into [min_x, max_x] and [min_y, max_y]
  target_x1 = std::clamp(request->x1, min_x, max_x);
  target_y1 = std::clamp(request->y1, min_y, max_y);
  target_x2 = std::clamp(request->x2, min_x, max_x);
  target_y2 = std::clamp(request->y2, min_y, max_y);
  target_x3 = std::clamp(request->x3, min_x, max_x);
  target_y3 = std::clamp(request->y3, min_y, max_y);

  RCLCPP_INFO(
    lg,
    "Clamped coords:   R:(%f, %f), G:(%f, %f), B:(%f, %f)",
    target_x1, target_y1,
    target_x2, target_y2,
    target_x3, target_y3);

  response->success = true;
  response_check = true;
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
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Add camera collision object
  create_camera_object(move_group, planning_scene_interface, logger);

  // Add table collision object
  create_table_object(move_group, planning_scene_interface, logger);

  // Start get_tcp_pos service
  rclcpp::Service<object_reference_msg::srv::ObjectReference>::SharedPtr service =
   node->create_service<object_reference_msg::srv::ObjectReference>("get_tcp_pos",  &get_tcp_pos_callback);

  RCLCPP_INFO(logger, "About to move to home position…");

  // Move robot to home pos
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForHome, logger, "home position");

  RCLCPP_INFO(logger, "Home position reached, now moving to reference position…");

  // MOve robot to first ref point
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  // Spin/wait for callback to set the flag
  while (!response_check) {
    int i = 0;
    for (i<=2; i++) {
      rclcpp::spin_some(node);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for ref pos");
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef2, logger, "reference position");
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");
      std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 
    }
    if (i == 2) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not all cubes found, shutting down");
        rclcpp::shutdown();
    }
  }

  response_check = false;

  // Create positions for the three cubes with callback parameters
  ReferencePosition firstPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x1, target_y1, position_z);
  geometry_msgs::msg::Pose firstPose = firstPosition.getPose();

  ReferencePosition secondPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x2, target_y2, position_z);
  geometry_msgs::msg::Pose secondPose = secondPosition.getPose();

  ReferencePosition thirdPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x3, target_y3, position_z);
  geometry_msgs::msg::Pose thirdPose = thirdPosition.getPose();

  // Move above Object positions
  RCLCPP_INFO(logger, "About to move to target 1");
  plan_and_execute.plan_and_execute_tcp(move_group, firstPose, logger, "target position1");

  RCLCPP_INFO(logger, "Home position reached, now moving to reference position…");

  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  RCLCPP_INFO(logger, "About to move to target 2");
  plan_and_execute.plan_and_execute_tcp(move_group, secondPose, logger, "target position2");

  RCLCPP_INFO(logger, "Home position reached, now moving to reference position…");

  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  RCLCPP_INFO(logger, "About to move to target 3");

  plan_and_execute.plan_and_execute_tcp(move_group, thirdPose, logger, "target position3");

  RCLCPP_INFO(logger, "Home position reached, now moving to reference position…");

  // MOve robot to first ref point
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  rclcpp::shutdown();
  return 0;
}

