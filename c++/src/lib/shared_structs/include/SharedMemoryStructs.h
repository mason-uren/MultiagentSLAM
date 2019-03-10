//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SHAREDSTRUCT_H
#define C_SHAREDSTRUCT_H

#include "SLAMConfigIn.h"

#define FILTER_LENGTH 64
#define RANGE_SENSOR_COUNT 3
#define MAX_FEATURES_IN_SET 3
#define MAX_AGENTS 6

/**
 * Enumerations
 */

enum pos_val { X = 0, Y, THETA };
enum ray_val { RANGE = 0, ANGLE = 1};

enum class node_color {
    BLACK = 0,
    RED
};

enum class descriptor {
    X = 0,
    Y,
    THETA,
    FEAT
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
    float xRelative;
    float yRelative;
    float incidentRay;
} FEATURE;

typedef struct {
    descriptor type;
    union {
        float poseComponent;
        FEATURE feature;
    };
} ELEMENT;

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
    float theta;
} POSE;

typedef struct {
    JSON_CONFIG config;
    long block_id{};
} SYS_CONFIG_IN;


#endif //C_SHAREDSTRUCT_H
