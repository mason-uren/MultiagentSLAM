//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SHAREDSTRUCT_H
#define C_SHAREDSTRUCT_H

#include "SLAMConfigIn.h"

constexpr uint16_t FILTER_LENGTH = UINT16_C(64);
constexpr uint16_t RANGE_SENSOR_COUNT = UINT16_C(3);
constexpr uint16_t FEATURE_LIMIT = UINT16_C(3);
constexpr uint16_t SIGNATURE_MAX = UINT16_C(100);
constexpr uint16_t ELEMENT_SIZE = UINT16_C(3);
constexpr float ROS_INTERVAL = 0.1;


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
    uint16_t idx{}; // TODO I don't like this here (doesn't relate to feature)
    float correspondence{};
    RAY incidentRay{};
    POSE pose{};
} FEATURE;

typedef struct {
    JSON_CONFIG config;
    long block_id{};
} SYS_CONFIG_IN;

template<typename Enum>
constexpr typename std::underlying_type<Enum>::type num(const Enum &anEnum) noexcept {
    return static_cast<typename std::underlying_type<Enum>::type>(anEnum);
};


#endif //C_SHAREDSTRUCT_H
