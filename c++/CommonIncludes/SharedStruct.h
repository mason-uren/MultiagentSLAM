//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SHAREDSTRUCT_H
#define C_SHAREDSTRUCT_H

/**
 * Enumerations
 */

enum class dir {
    LEFT = 0,
    RIGHT
} DIRECTION;

enum class node_color {
    BLACK = 0,
    RED
} NODE_COLOR;

enum class descriptor {
    X = 0,
    Y,
    THETA,
    FEAT
} DESCRIPTOR;

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
    float indcidentRay;
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
    bool valid;
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


#endif //C_SHAREDSTRUCT_H
