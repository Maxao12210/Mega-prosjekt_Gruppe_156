//
// Created by marcin on 5/27/25.
//

#ifndef PATHPLANNING_HPP
#define PATHPLANNING_HPP

class PathPlanning {
    public:

    PathPlanning() = default;

    // Helper function to plan and execute joint movements
    bool plan_and_execute_joint(
      moveit::planning_interface::MoveGroupInterface & move_group,
      const std::map<std::string, double> & joint_targets,
      const rclcpp::Logger & logger,
      const std::string & description)
    {
        /*moveit_msgs::msg::JointConstraint joint_constraint;
        joint_constraint.joint_name = "shoulder_lift_joint";
        joint_constraint.position = -0.79;       // Center of allowed range
        joint_constraint.tolerance_below = 0.79; // 0.79 rad below
        joint_constraint.tolerance_above = 0.79; // 0.79 rad above
        joint_constraint.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.joint_constraints.push_back(joint_constraint);

        // Apply to MoveGroupInterface
        move_group.setPathConstraints(path_constraints);*/

        move_group.setJointValueTarget(joint_targets);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group.plan(plan));
        if (success) {
            RCLCPP_INFO(logger, "Planning to %s successful, executing...", description.c_str());
            move_group.execute(plan);
            //move_group.clearPathConstraints();
            return true;
        } else {
            RCLCPP_ERROR(logger, "Planning to %s failed!", description.c_str());
            //move_group.clearPathConstraints();
            rclcpp::shutdown();
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
        /*moveit_msgs::msg::JointConstraint jc_shoulder_lift;
        jc_shoulder_lift.joint_name = "shoulder_lift_joint";
        jc_shoulder_lift.position = -0.79;       // Center of allowed range
        jc_shoulder_lift.tolerance_below = 0.79; // 0.79 rad below
        jc_shoulder_lift.tolerance_above = 0.79; // 0.79 rad above
        jc_shoulder_lift.weight = 1.0;

        moveit_msgs::msg::JointConstraint jc_wrist_2;
        jc_wrist_2.joint_name = "wrist_2_joint";
        jc_wrist_2.position = -1.57;
        jc_wrist_2.tolerance_below = 0.1;
        jc_wrist_2.tolerance_above = 0.1;
        jc_wrist_2.weight = 1.0;

        moveit_msgs::msg::JointConstraint jc_shoulder_pan;
        jc_shoulder_pan.joint_name = "shoulder_pan_joint";
        jc_shoulder_pan.position = -0.393;
        jc_shoulder_pan.tolerance_below = 1.178;
        jc_shoulder_pan.tolerance_above = 1.178;
        jc_shoulder_pan.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.joint_constraints.push_back(jc_shoulder_lift);
        path_constraints.joint_constraints.push_back(jc_wrist_2);
        path_constraints.joint_constraints.push_back(jc_shoulder_pan);

        // Apply to MoveGroupInterface
        move_group.setPathConstraints(path_constraints);*/

        move_group.setPoseTarget(target_pos);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group.plan(plan));
        if (success) {
            RCLCPP_INFO(logger, "Planning to %s successful, executing...", description.c_str());
            move_group.execute(plan);
            move_group.clearPathConstraints();
            return true;
        } else {
            RCLCPP_ERROR(logger, "Planning to %s failed!", description.c_str());
            move_group.clearPathConstraints();
            rclcpp::shutdown();
            return false;
        }
    }

    ~PathPlanning() = default;
};

#endif //PATHPLANNING_HPP


