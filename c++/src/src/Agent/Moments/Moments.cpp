//
// Created by Mason U'Ren on 2019-02-20.
//

#include "Moments.h"

std::array<MeanFilter<float> *, 3> *Moments::getMeans() {
    return &this->means;
}

std::array<VarianceFilter<float> *, 3> *Moments::getVariances() {
    return &this->variances;
}
