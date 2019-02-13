//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_ROVERINTERFACE_H
#define C_ROVERINTERFACE_H

#include <string>
#include "../CommonIncludes/SharedStruct.h"

class RoverInterface {
    virtual ~RoverInterface() = default;

public:
    virtual int getID() = 0;
    virtual std::string getName() = 0;
    virtual VELOCITY getVelocity() = 0;

protected:
    virtual void setName(std::string name) = 0;
    virtual void setVelocity(VELOCITY velocity) = 0;
};

#endif //C_ROVERINTERFACE_H
