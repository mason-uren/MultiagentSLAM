//
// Created by Mason U'Ren on 2019-04-15.
//

#ifndef MULTIAGENTSLAM_ADAPTER_H
#define MULTIAGENTSLAM_ADAPTER_H

#include <memory>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cmath>
#include <Eigen/Dense>

#include <MeanFilter.h>
#include <VarianceFilter.h>
#include <CovarianceFilter.h>
#include <Matrix.h>

#include "Agent/Rover/RoverFactory.h"
#include "Slam/SlamAdapter/SlamAdapter.h"
#include "Slam/FeatureSet/FeatureSet.h"
#include "Utilities/SharedMemory/SharedMemory.h"
#include "Utilities/ConfigParser/ConfigParser.h"
#include "Utilities/Equations/Equations.h"

// Necessary functions
void loadDefaultConfig();
void jsonInitialize();
void kinematicHandler(const POSE &pose, const VELOCITY &vel); // TODO will need to change for ROS
void sonarHandler(const std::array<SONAR, 3> &sonar); // TODO will need to change for ROS
void slamHandler();
//void auxilaryRoverHandler(const AuxilaryBeliefs &auxBeliefs);
//void featureSetHandler(const AuxilaryFeatureSets &auxFS);
//void transformationHandler(const TransformationPairs &transPairs);
void publishFeatureSet(const std::array<FEATURE, FEATURE_LIMIT> &featureSet, const CLASSIFIER &classifer);
void publishBelief(const POSE &pose, const float &confi);
void publishTransformation(const POSE &trans, const string &rID);

void testEnv();

// Verification Functions
void printActiveRovers();
void testMeanFilter();
void testVarianceFilter();
void testFIRFilter();
void testTransformations();
void testMoments();
void testEquations();
void testRedBlackTree();
void testDetections();
void testMatrix();
void testMatrixMan();
void testSeif();
void testFeatureSet();

static unique_ptr<SharedMemory> sharedMemory;
static unique_ptr<ConfigParser> configParser;
static unique_ptr<Seif> seif;
static unique_ptr<Detection> detection;
static unique_ptr<RedBlackTree> localMap;
static unique_ptr<SYS_CONFIG_IN> systemConfig;

//static Rover rover{};
static string roverName;
static bool canReadLocation;






#endif //MULTIAGENTSLAM_ADAPTER_H
