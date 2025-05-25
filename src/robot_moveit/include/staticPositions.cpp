//
// Created by drinkalotofwater on 5/25/25.
//

#include "staticPositions.h"


staticPositions homePosition("Home position", 0.000,-1.571,-0.000,0.000,0.000,0.000);
std::map<std::string, double> coordinatesForHome = homePosition.getCoordinatesMap();

staticPositions ref1("Position nr.1", 0.005, -1.060, 0.3875, -0.930, -1.474,-0.0005);
std::map<std::string, double> coordinatesForRef1 = ref1.getCoordinatesMap();