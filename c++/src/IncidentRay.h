//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_INCIDENTRAY_H
#define C_INCIDENTRAY_H

#include "../CommonIncludes/SharedStruct.h"

class RoverInterface {
    virtual ~RoverInterface() = default;

public:
    virtual bool hasIncidentRay() = 0;
    virtual RAY getIncidentRay() = 0;

protected:
    virtual void setIncidentRay(RAY ray) = 0;
};

#endif //C_INCIDENTRAY_H
