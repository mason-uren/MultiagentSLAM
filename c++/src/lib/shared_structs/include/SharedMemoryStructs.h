//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SHAREDSTRUCT_H
#define C_SHAREDSTRUCT_H

#include "SLAMConfigIn.h"

#define FILTER_LENGTH 64
#define RANGE_SENSOR_COUNT 3
#define FEATURE_LIMIT 3
#define SIGNATURE_MAX 100
#define ELEMENT_SIZE 3
#define ROS_INTERVAL 0.1


/**
 * Enumerations
 */

enum class pos_val {
    X = 0,
    Y,
    THETA
};
enum class measurement {
    RANGE = 0,
    ANGLE = 1,
    CORRESPONDENCE = 2
};

enum class cov_idx {
    XY = 0,
    XZ = 1,
    YZ = 2,
};

enum class node_color {
    BLACK = 0,
    RED
};

enum class sonar_id {
    LEFT = 0,
    CENTER,
    RIGHT
};

/**
 * Structs
 */

typedef struct {
    float area;
    float orientation;
    float signature;
} CLASSIFIER;

typedef struct {
    sonar_id id;
    float observedRange;
} SONAR;

typedef struct {
    float range;
    float angle;
} RAY;

typedef struct {
    float linear;
    float angular;
} VELOCITY;

typedef struct {
    float x;
    float y;
} LOCATION;

typedef struct {
    float x;
    float y;
    float theta;
} POSE;

typedef struct {
    unsigned long idx{}; // TODO I don't like this here (doesn't relate to feature)
    float correspondence{};
    RAY incidentRay{};
    POSE pose{};
} FEATURE;

typedef struct {
    JSON_CONFIG config;
    long block_id{};
} SYS_CONFIG_IN;


#endif //C_SHAREDSTRUCT_H
