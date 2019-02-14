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
    unsigned long maxFeatures;
} SEIF_CONFIG;

typedef struct {
    bool valid;
    float highDetectionBoundaryInM;
    float sonarCoverageInRad;
    float sonarRangeInM;
} DETECTION_CONFIG;

typedef struct {
    bool valid{};
    bool live{};
    int ID{};
    std::string name;
    DETECTION_CONFIG detectionConfig;
    SEIF_CONFIG seifConfig;
} ROVER_CONFIG;

typedef struct {
    bool valid{};
    int numberOfRovers{};
    int filterSize{};
    ROVER_CONFIG rovers[6];
} SLAM_CONFIG;

typedef struct {
    SLAM_CONFIG slamConfig;
    size_t hash{};
} JSON_CONFIG;




#endif //C_SLAMCONFIGIN_H
