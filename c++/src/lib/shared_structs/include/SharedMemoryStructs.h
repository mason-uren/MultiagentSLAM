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
#define No_RAY_ELEMS 2
#define No_POSE_ELEMS 3

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
    STATE_ESTIMATE = 0,
    FOUND_FEATURE
};

enum class sonar_id {
    LEFT = 0,
    CENTER,
    RIGHT
};

enum class obj_type {
    UNDEFINED = 0,
    LONG,
    FLOAT,
    OBJECT
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

// IMPORTANT : must always exist at index 0, within information matrix/vector
typedef struct {
    float x;
    float y;
    float theta;
} POSE;

typedef struct {
    float xRelative;
    float yRelative;
    RAY incidentRay;
} FEATURE;

//typedef struct {
//    descriptor type;
//    union {
//        POSE pose;
//        FEATURE feature;
//    };
//} ELEMENT;

typedef struct {
    JSON_CONFIG config;
    long block_id{};
} SYS_CONFIG_IN;


#endif //C_SHAREDSTRUCT_H
