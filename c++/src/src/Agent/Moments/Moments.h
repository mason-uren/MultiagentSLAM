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
#include <CovarianceFilter.h>
#include <SharedMemoryStructs.h>

typedef struct {
    std::array<MeanFilter<float> *, ELEMENT_SIZE> means {
            new MeanFilter<float>(),
            new MeanFilter<float>(),
            new MeanFilter<float>()
    };
    std::array<VarianceFilter<float> *, ELEMENT_SIZE> variances {
            new VarianceFilter<float>(),
            new VarianceFilter<float>(),
            new VarianceFilter<float>()
    };
    std::array<CovarianceFilter<float> *, ELEMENT_SIZE> covariances {
            new CovarianceFilter<float>(),
            new CovarianceFilter<float>(),
            new CovarianceFilter<float>()
    };
} STATS;


class Moments {
public:
    static Moments *getInstance() {
        static Moments instance;
        return &instance;
    }

    STATS * getMotion();
    STATS * getMeasurement();

private:
    Moments() :
            motion(new STATS{}),
            measurement(new STATS{})
    {}
    Moments(Moments const &);
    void operator=(Moments const &);

    std::shared_ptr<STATS> motion;
    std::shared_ptr<STATS> measurement;



};


#endif //MULTIAGENTSLAM_MOMENTS_H
