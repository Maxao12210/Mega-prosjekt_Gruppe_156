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

    // Helper function to plan and execute tcp position
    bool plan_and_execute_tcp(
      moveit::planning_interface::MoveGroupInterface & move_group,
      const geometry_msgs::msg::Pose & target_pos,
      const rclcpp::Logger & logger,
      const std::string & description)
    {
        move_group.setPoseTarget(target_pos);

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

    ~PathPlanning() = default;
};

#endif //PATHPLANNING_HPP
