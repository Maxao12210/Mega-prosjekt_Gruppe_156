//
// Created by drinkalotofwater on 5/25/25.
//

#ifndef STATICPOSITIONS_HPP
#define STATICPOSITIONS_HPP
#include <iostream>
#include <map>



class StaticPositions  {
    public:
        std::string name;
        double shoulderPanJoint;
        double shoulderLiftJoint;
        double elbowJoint;
        double wrist1Joint;
        double wrist2Joint;
        double wrist3Joint;
        std::map<std::string, double> coordinatesMap;


    StaticPositions(std::string nam, double sPan = 0.000, double sLift = 0.000, double elb = 0.000, double w1 = 0.000,double w2 = 0.000,double w3 = 0.000)
    :name(std::move(nam)), shoulderPanJoint(sPan), shoulderLiftJoint(sLift), elbowJoint(elb), wrist1Joint(w1), wrist2Joint(w2), wrist3Joint(w3)
    {
        coordinatesMap["shoulder_pan_joint"] = shoulderPanJoint;
        coordinatesMap["shoulder_lift_joint"] = shoulderLiftJoint;
        coordinatesMap["elbow_joint"] = elbowJoint;
        coordinatesMap["wrist_1_joint"] = wrist1Joint;
        coordinatesMap["wrist_2_joint"] = wrist2Joint;
        coordinatesMap["wrist_3_joint"] = wrist3Joint;
    }

    [[nodiscard]] std::map<std::string, double> getCoordinatesMap () const {
        return coordinatesMap;
    }

    ~StaticPositions()=default;
};



#endif //STATICPOSITIONS_HPP
