//
// Created by Mason U'Ren on 2019-02-20.
//

#ifndef MULTIAGENTSLAM_MOMENTS_H
#define MULTIAGENTSLAM_MOMENTS_H


#include <iostream>
#include <sstream>
#include <array>

#include "../../Utilities/Filters/MeanFilter.h"
#include "../../Utilities/Filters/VarianceFilter.h"

enum pos_val { X = 0, Y, THETA };

class Moments {
public:
    Moments() :
        means{
            new MeanFilter<float>(),
            new MeanFilter<float>(),
            new MeanFilter<float>()
        },
        variances{
            new VarianceFilter<float>(),
            new VarianceFilter<float>(),
            new VarianceFilter<float>()
        }
    {}
    ~Moments() = default;

    std::array<MeanFilter<float> *, 3> *getMeans();
    std::array<VarianceFilter<float> *, 3> *getVariances();

private:
    std::array<MeanFilter<float> *, 3> means;
    std::array<VarianceFilter<float> *, 3> variances;

};


#endif //MULTIAGENTSLAM_MOMENTS_H
