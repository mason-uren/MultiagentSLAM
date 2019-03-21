//
// Created by Mason U'Ren on 2019-02-20.
//

#ifndef MULTIAGENTSLAM_MOMENTS_H
#define MULTIAGENTSLAM_MOMENTS_H


#include <iostream>
#include <sstream>
#include <array>

#include <MeanFilter.h>
#include <VarianceFilter.h>

class Moments {
public:
    static Moments *getInstance() {
        static Moments instance;
        return &instance;
    }

    std::array<MeanFilter<float> *, 3> *getMeans();
    std::array<VarianceFilter<float> *, 3> *getVariances();

private:
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
    Moments(Moments const &);
    void operator=(Moments const &);

    std::array<MeanFilter<float> *, 3> means;
    std::array<VarianceFilter<float> *, 3> variances;

};


#endif //MULTIAGENTSLAM_MOMENTS_H
