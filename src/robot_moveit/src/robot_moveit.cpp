#include <algorithm>  // for std::min, std::max
#include <memory>
#include "staticPositions.h"
#include "referencePosition.hpp"
#include "PathPlanning.hpp"
#include  "CameraBracket.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
//#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "object_reference_msg/srv/object_reference.hpp"

// Source link: https://moveit.picknik.ai/main/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html

PathPlanning plan_and_execute;

staticPositions homePosition("Home position", 0.000,-1.571,-0.000,0.000,0.000,0.000);
std::map<std::string, double> coordinatesForHome = homePosition.getCoordinatesMap();

staticPositions ref1("Position nr.1", 0.005, -1.060, 0.3875, -0.930, -1.57,-0.0005);
std::map<std::string, double> coordinatesForRef1 = ref1.getCoordinatesMap();

// Not accurate values
double orientation_x = 0.709, orientation_y = -0.704, orientation_z = -0.023, orientation_w = -0.045;
double target_x1, target_y1, target_x2, target_y2, target_x3, target_y3;
double position_z = 0.25;

double min_x = 0.20;
double min_y = -0.13;
double max_x = 0.6;
double max_y = 0.45;

bool response_check = false;

// Get tcp pos values from get_tcp_pos service
/*void get_tcp_pos(const std::shared_ptr<object_reference_msg::srv::ObjectReference::Request> &request,
                 const std::shared_ptr<object_reference_msg::srv::ObjectReference::Response> &response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx1: %f" " y1: %f" "x2: %f" "y2: %f" "x3: %f" "y3: %f",
                  request->x1, request->y1, request->x2, request->y2, request->x3, request->y3);

    if ((min_x <= request->x1 && request->x1 <= max_x) &&
        (min_x <= request->x2 && request->x2 <= max_x) &&
        (min_x <= request->x3 && request->x3 <= max_x) &&
        (min_y <= request->y1 && request->y1 <= max_y) &&
        (min_y <= request->y2 && request->y2 <= max_y) &&
        (min_y <= request->y3 && request->y3 <= max_y)) // Usikre verdier
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
}*/
void get_tcp_pos(
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

  create_table_object(move_group, planning_scene_interface, logger);


  // Start get_tcp_pos service
  rclcpp::Service<object_reference_msg::srv::ObjectReference>::SharedPtr service =
   node->create_service<object_reference_msg::srv::ObjectReference>("get_tcp_pos",  &get_tcp_pos);

  RCLCPP_INFO(logger, "About to move to home position…");

  // Move robot to home pos
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForHome, logger, "home position");

  RCLCPP_INFO(logger, "Home position reached, now moving to reference position…");

  // MOve robot to first ref point
  plan_and_execute.plan_and_execute_joint(move_group, coordinatesForRef1, logger, "reference position");

  // Spin/wait for callback to set the flag
  while (!response_check) {
    rclcpp::spin_some(node);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for ref pos");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // avoid busy loop
  }

  response_check = false;

  referencePosition firstPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x1, target_y1, position_z);
  geometry_msgs::msg::Pose firstPose = firstPosition.getPose();

  referencePosition secondPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x2, target_y2, position_z);
  geometry_msgs::msg::Pose secondPose = secondPosition.getPose();

  referencePosition thirdPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x3, target_y3, position_z);
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

