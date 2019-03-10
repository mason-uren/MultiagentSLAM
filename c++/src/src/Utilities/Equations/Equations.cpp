//
// Created by Mason U'Ren on 2019-02-21.
//

#include "Equations.h"

// RAY - range (0) angle (1)
// POSE - x (0) y (1) theta (2)
std::array<float, 2> Equations::originToPoint(const std::array<float, 2> &ray, const std::array<float, 3> &pose) {
    return {
            (ray[RANGE] * sin(pose[THETA] + ray[ANGLE]) + pose[X]),
            (ray[RANGE] * cos(pose[THETA] + ray[ANGLE]) + pose[Y])
    };
}

float Equations::wrapTheta(const double &orientation) {
    float heading = (float) orientation;
    if (fmod(heading + M_PI, 2 * M_PI) < 0) {
        heading += 2 * M_PI;
    }
    return (-1) * static_cast<float>(heading - M_PI);
}

float Equations::normalizeValue(const float &value, const float &lowbound, const float &highbound) {
    return (value - lowbound) / (highbound - lowbound);
}

std::array<float, 2> Equations::centroid(const std::array<std::array<float, 2>, 3> &coordinatePairs) {
    return {
            (coordinatePairs[0][X] + coordinatePairs[1][X] + coordinatePairs[2][X]) / 3,
            (coordinatePairs[0][Y] + coordinatePairs[1][Y] + coordinatePairs[2][Y]) / 3
    };
}

float Equations::cantor(const float &val_1, const float &val_2) {
    return val_2 + (val_1 + val_2) * (val_1 + val_2 + 1) / 2;
}

float Equations::straightAvg(const std::vector<float> &toAvg) {
    float total = 0;
    for (auto value : toAvg) {
        total += value;
    }
    return total / toAvg.size();
}

float Equations::dotProduct(const std::vector<float> *vec_1, const std::vector<float> *vec_2) {
    float dotProduct = 0;
    if (vec_1 && vec_2) {
        for (unsigned long i = 0; i < vec_1->size(); i++) {
            dotProduct += (*vec_1)[i] * (*vec_2)[i];
        }
    }
    return dotProduct;
}
