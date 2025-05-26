//
// Created by marcin on 5/26/25.
//

#include "referencePosition.hpp"
#include "object_reference_msg/srv/object_reference.hpp"

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

referencePosition firstPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x1, target_y1, position_z);
geometry_msgs::msg::Pose firstPose = firstPosition.getPose();

referencePosition secondPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x2, target_y2, position_z);
geometry_msgs::msg::Pose secondPose = secondPosition.getPose();

referencePosition thirdPosition(orientation_x, orientation_y, orientation_z, orientation_w, target_x3, target_y3, position_z);
geometry_msgs::msg::Pose thirdPose = thirdPosition.getPose();