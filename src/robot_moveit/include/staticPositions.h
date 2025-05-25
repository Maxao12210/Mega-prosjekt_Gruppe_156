//
// Created by drinkalotofwater on 5/25/25.
//

#ifndef STATICPOSITIONS_H
#define STATICPOSITIONS_H
#include <iostream>



class staticPositions  {
    public:
        std::string name = "Static position";
        double shoulderPanJoint = 0.0;
        double shoulderLiftJoint = 0.0;
        double elbowJoint = 0.0;
        double wrist1Joint = 0.0;
        double wrist2Joint = 0.0;
        double wrist3Joint = 0.0;

    staticPositions(std::string, double, double, double, double);

    void changeValues (const std::string& newName, double shoulderPan, double shoulderLift, double elbow, double wrist1, double wrist2, double wrist3){
        name = newName;
        shoulderPanJoint = shoulderPan;
        shoulderLiftJoint = shoulderLift;
        elbowJoint = elbow;
        wrist1Joint = wrist1;
        wrist2Joint = wrist2;
        wrist3Joint = wrist3;

        std::cout<<"The new valus are: \n" << shoulderPanJoint << "\n" << shoulderLiftJoint << "\n" << elbowJoint << "\n" << wrist1Joint << "\n" << wrist2Joint << "\n" << wrist3Joint << std::endl;
    }
    ~staticPositions()=default;
};



#endif //STATICPOSITIONS_H
