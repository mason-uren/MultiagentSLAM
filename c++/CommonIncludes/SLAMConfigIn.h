//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SLAMCONFIGIN_H
#define C_SLAMCONFIGIN_H

#include <string>

/**
 * Structs
 */

typedef struct {
    bool valid;
    int maxActiveFeatures;
    long maxFeatures;
} SEIF_CONFIG;

typedef struct {
    bool valid;
    std::string name;
} ROVER_CONFIG;

typedef struct {
    bool valid;
    int numberOfRovers;
} WORLD_CONFIG;

typedef struct {
    bool valid;
    float highDetectionBoundary;
    float sonarCoverage;
    float sonarRangeInM;
    int filterSize;
} DETECTION_CONFIG;


#endif //C_SLAMCONFIGIN_H
