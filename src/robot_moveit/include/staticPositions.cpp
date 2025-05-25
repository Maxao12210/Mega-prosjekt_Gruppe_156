//
// Created by drinkalotofwater on 5/25/25.
//

#include "staticPositions.h"

int main() {

    staticPositions homePosition("Home position", 0.0,-1.571,-0,0.0,0.0,0.0);
    double shoulderLiftJointHome = homePosition.shoulderLiftJoint;
    double shoulderPanJointHome = homePosition.shoulderPanJoint;
    double wrist1Home = homePosition.wrist1Joint;
    double wrist2Home = homePosition.wrist2Joint;
    double wrist3Home = homePosition.wrist3Joint;

    staticPositions ref1("Position nr.1", 0.005, -1.060, 0.3875, -0.930, -1.474,-0.0005);
    double shoulderLiftJointRef1 = ref1.shoulderLiftJoint;
    double shoulderPanJointRef1 = ref1.shoulderPanJoint;
    double wrist1Ref1 = ref1.wrist1Joint;
    double wrist2Ref1 = ref1.wrist2Joint;
    double wrist3Ref1 = ref1.wrist3Joint;

    return 0;
}