//
// Created by marcin on 5/25/25.
//

#ifndef STATIC_POS_HPP
#define STATIC_POS_HPP

#include <moveit/move_group_interface/move_group_interface.hpp>

class referencePosition {
    public:
        double orientation_x;
        double orientation_y;
        double orientation_z;
        double orientation_w;
        double position_x;
        double position_y;
        double position_z;
        geometry_msgs::msg::Pose pose;

    referencePosition(double or_x, double or_y, double or_z, double or_w, double pos_x, double pos_y, double pos_z)
       : orientation_x(or_x), orientation_y(or_y), orientation_z(or_z), orientation_w(or_w), position_x(pos_x), position_y(pos_y), position_z(pos_z)
    {
       pose.orientation.x = orientation_x;
       pose.orientation.y = orientation_y;
       pose.orientation.z = orientation_z;
       pose.orientation.w = orientation_w;
       pose.position.x = position_x;
       pose.position.y = position_y;
       pose.position.z = position_z;
    }

    geometry_msgs::msg::Pose getPose() const {
        return pose;
    }

    ~referencePosition()=default;
};

#endif //STATIC_POS_HPP
