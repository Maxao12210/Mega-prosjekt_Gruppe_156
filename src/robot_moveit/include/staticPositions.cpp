//
// Created by drinkalotofwater on 5/25/25.
//

#include "staticPositions.h"

int main() {

    staticPositions homePosition("Home position", 0.0,-1.571,-0,0.0,0.0,0.0);
    homePosition.getCoordinatesMap();

    staticPositions ref1("Position nr.1", 0.005, -1.060, 0.3875, -0.930, -1.474,-0.0005);
    ref1.getCoordinatesMap();

    return 0;
}