//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_TRANSFORMATION_H
#define C_TRANSFORMATION_H

//#lib <SharedMemoryStructs.h>

#include <vector>
#include <SLAMConfigIn.h>
#include <FIRFilter.h>

class Transformation {
public:
    Transformation() :
        x_translation(new FIRFilter<float, FILTER_LENGTH>(xBuf)),
        y_translation(new FIRFilter<float, FILTER_LENGTH>(yBuf)),
        orientation(new FIRFilter<float, FILTER_LENGTH>(orientBuf))
    {}
    virtual ~Transformation() = default;

    FIRFilter<float, FILTER_LENGTH> *x_translation; // = FIRFilter<float, FILTER_LENGTH>(xBuf);
    FIRFilter<float, FILTER_LENGTH> *y_translation; // = FIRFilter<float, FILTER_LENGTH>(yBuf);
    FIRFilter<float, FILTER_LENGTH> *orientation; // = FIRFilter<float, FILTER_LENGTH>(orientBuf);

private:
    float xBuf[FILTER_LENGTH]{};
    float yBuf[FILTER_LENGTH]{};
    float orientBuf[FILTER_LENGTH]{};
};

#endif //C_TRANSFORMATION_H