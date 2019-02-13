//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_BELIEFINTERFACE_H
#define C_BELIEFINTERFACE_H

#include "../CommonIncludes/SharedStruct.h"

class BeliefInterface {
    virtual ~BeliefInterface() = default;

public:
    virtual POSE getPose() = 0;
    virtual float getConfidence() = 0;

protected:
    virtual void setPose(POSE pose)= 0;
    virtual void setConfidence(float confi) = 0;
};

#endif //C_BELIEFINTERFACE_H
