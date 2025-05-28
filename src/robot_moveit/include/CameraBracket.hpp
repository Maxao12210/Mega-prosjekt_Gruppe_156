//
// Created by marcin on 5/28/25.
//

#ifndef CAMERABRACKET_HPP
#define CAMERABRACKET_HPP
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

bool create_camera_object(
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const rclcpp::Logger & logger
) {
        moveit_msgs::msg::CollisionObject camera_object;

        camera_object.id = "camera";
        shape_msgs::msg::SolidPrimitive camera_primitives;
        camera_primitives.type = camera_primitives.BOX;
        camera_primitives.dimensions.resize(3);
        camera_primitives.dimensions[camera_primitives.BOX_X] = 0.07;
        camera_primitives.dimensions[camera_primitives.BOX_Y] = 0.12;
        camera_primitives.dimensions[camera_primitives.BOX_Z] = 0.1;

        camera_object.header.frame_id = move_group.getEndEffectorLink();
        geometry_msgs::msg::Pose grab_pose;
        grab_pose.orientation.w = 1.0;
        grab_pose.position.y = -0.06;
        grab_pose.position.z = 0.05;

        camera_object.primitives.push_back(camera_primitives);
        camera_object.primitive_poses.push_back(grab_pose);
        camera_object.operation = camera_object.ADD;
        planning_scene_interface.applyCollisionObject(camera_object);

        RCLCPP_INFO(logger, "Attach the object to the robot");
        std::vector<std::string> touch_links;
        touch_links.push_back("tool0");
        move_group.attachObject(camera_object.id, "wrist_3_link", touch_links);

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(camera_object);

        return true;
    }


bool create_table_object(
    moveit::planning_interface::MoveGroupInterface &move_group,
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const rclcpp::Logger & logger
    ) {
    moveit_msgs::msg::CollisionObject table_object;
    table_object.id = "table";
    table_object.header.frame_id = move_group.getPlanningFrame();

    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[table_primitive.BOX_X] = 1.2;
    table_primitive.dimensions[table_primitive.BOX_Y] = 0.85;
    table_primitive.dimensions[table_primitive.BOX_Z] = 0.02;

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.5;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.15;

    table_object.primitives.push_back(table_primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(table_object);

    RCLCPP_INFO(logger, "Add an table into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    return true;
    }

#endif //CAMERABRACKET_HPP
