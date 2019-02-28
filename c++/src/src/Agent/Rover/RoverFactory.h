//
// Created by Mason U'Ren on 2019-02-14.
//

#ifndef C_ROVERFACTORY_H
#define C_ROVERFACTORY_H


#include <SLAMConfigIn.h>
#include "Rover.h"

class RoverFactory {
public:
    static Rover *create(ROVER_CONFIG *roverConfig);
};


#endif //C_ROVERFACTORY_H
