//
// Created by drinkalotofwater on 5/25/25.
//

#ifndef STATICPOSITIONS_H
#define STATICPOSITIONS_H
#include <iostream>
#include <map>



class staticPositions  {
    public:
        std::string name;
        double shoulderPanJoint;
        double shoulderLiftJoint;
        double elbowJoint;
        double wrist1Joint;
        double wrist2Joint;
        double wrist3Joint;
        std::map<std::string, double> coordinatesMap;


    staticPositions(std::string nam, double sPan = 0.000, double sLift = 0.000, double elb = 0.000, double w1 = 0.000,double w2 = 0.000,double w3 = 0.000)
    :name(std::move(nam)), shoulderPanJoint(sPan), shoulderLiftJoint(sLift), elbowJoint(elb), wrist1Joint(w1), wrist2Joint(w2), wrist3Joint(w3)
    {
        coordinatesMap["shoulder_pan_joint"] = shoulderPanJoint;
        coordinatesMap["shoulder_lift_joint"] = shoulderLiftJoint;
        coordinatesMap["elbow_joint"] = elbowJoint;
        coordinatesMap["wrist_1_joint"] = wrist1Joint;
        coordinatesMap["wrist_2_joint"] = wrist2Joint;
        coordinatesMap["wrist_3_joint"] = wrist3Joint;
    }
    // Used during the debuging process, to find the best values for the position of the robot, when the camera takes the picture.
    void changeValues (const std::string& newName, double shoulderPan, double shoulderLift, double elbow, double wrist1, double wrist2, double wrist3){
        name = newName;
        shoulderPanJoint = shoulderPan;
        shoulderLiftJoint = shoulderLift;
        elbowJoint = elbow;
        wrist1Joint = wrist1;
        wrist2Joint = wrist2;
        wrist3Joint = wrist3;

        coordinatesMap["shoulder_pan_joint"] = shoulderPanJoint;
        coordinatesMap["shoulder_lift_joint"] = shoulderLiftJoint;
        coordinatesMap["elbow_joint"] = elbowJoint;
        coordinatesMap["wrist_1_joint"] = wrist1Joint;
        coordinatesMap["wrist_2_joint"] = wrist2Joint;
        coordinatesMap["wrist_3_joint"] = wrist3Joint;

        std::cout<<"The new valus are: \n" << shoulderPanJoint << "\n" << shoulderLiftJoint << "\n" << elbowJoint << "\n" << wrist1Joint << "\n" << wrist2Joint << "\n" << wrist3Joint << std::endl;
    }

    [[nodiscard]] std::map<std::string, double> getCoordinatesMap () const {
        return coordinatesMap;
    }

    ~staticPositions()=default;
};



#endif //STATICPOSITIONS_H
