//
// Created by Mason U'Ren on 2019-04-15.
//

#ifndef MULTIAGENTSLAM_ROS_ADAPTER_H
#define MULTIAGENTSLAM_ROS_ADAPTER_H

#include <memory>

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
#include "Utilities/SharedMemory/SharedMemory.h"
#include "Utilities/ConfigParser/ConfigParser.h"
#include "Utilities/Equations/Equations.h"

//friend class FeatureSet;

// Necessary functions
void loadDefaultConfig();
void jsonInitialize();
void kinematicHandler();
void sonarHandler();
//void auxilaryRoverHandler(const AuxilaryBeliefs &auxBeliefs);
//void featureSetHandler(const AuxilaryFeatureSets &auxFS);
//void transformationHandler(const TransformationPairs &transPairs);
void publishFeatureSet(const std::array<FEATURE, FEATURE_LIMIT> &featureSet, const CLASSIFIER &classifer);
void publishTransformation(const Transformation &trans, const int &rID);

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

using json = nlohmann::json;
using std::shared_ptr;
using std::string;
using std::get;

static shared_ptr<SharedMemory> sharedMemory;
static shared_ptr<ConfigParser> configParser;
static shared_ptr<Seif> seif;
static shared_ptr<Detection> detection;
static shared_ptr<RedBlackTree> localMap;
static shared_ptr<SYS_CONFIG_IN> systemConfig;

static Rover rover{};
static string roverName;
static bool canReadLocation;






#endif //MULTIAGENTSLAM_ROS_ADAPTER_H
