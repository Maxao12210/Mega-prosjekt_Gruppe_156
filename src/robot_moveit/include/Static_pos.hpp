//
// Created by marcin on 5/25/25.
//

#ifndef STATIC_POS_HPP
#define STATIC_POS_HPP

#include <map>
#include <string>

// Define "home" joint positions
inline std::map<std::string, double> home_joint_values = {
  {"shoulder_pan_joint",    0.000},
  {"shoulder_lift_joint",  -1.571},
  {"elbow_joint",          -0.000},
  {"wrist_1_joint",         0.000},
  {"wrist_2_joint",         0.000},
  {"wrist_3_joint",         0.000}
};

// Define reference joint positions
inline std::map<std::string, double> ref_joint_values = {
  {"shoulder_pan_joint",    -0.012},
  {"shoulder_lift_joint",  -1.539},
  {"elbow_joint",           0.105},
  {"wrist_1_joint",        -0.298},
  {"wrist_2_joint",        -1.459},
  {"wrist_3_joint",        -0.0006}
};

#endif //STATIC_POS_HPP
