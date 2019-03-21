//
// Created by Mason U'Ren on 2019-02-21.
//

#ifndef MULTIAGENTSLAM_EQUATIONS_H
#define MULTIAGENTSLAM_EQUATIONS_H


#include <array>
#include <vector>
#include <numeric>
#include <math.h>
#include <SharedMemoryStructs.h>

class Equations {
public:
    static Equations *getInstance() {
        static Equations instance;
        return &instance;
    }

    std::array<float, 2> originToPoint(const std::array<float, 2> &ray, const std::array<float, 3> &pose);
    float wrapTheta(const double &orientation);
    float normalizeValue(const float &value, const float &lowbound, const float &highbound);
    std::array<float, 2> centroid(const std::array<std::array<float, 2>, 3> &coordinatePairs);
    float cantor(const float &val_1, const float &val_2);
    float straightAvg(const std::vector<float> &toAvg);
    float dotProduct(const std::vector<float> *vec_1, const std::vector<float> *vec_2);

private:
    Equations () = default;
    Equations(Equations const&);
    void operator=(Equations const&);
};


#endif //MULTIAGENTSLAM_EQUATIONS_H
