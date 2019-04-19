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

#include <shared_structs/include/SharedMemoryStructs.h>
#include <templates/include/SharedMemory/SharedMemory.h>

#include "Agent/Detections/Detection.h"
#include "Slam/SlamAdapter/SlamAdapter.h"
#include "Slam/Seif/Seif.h"
#include "Utilities/ConfigParser/ConfigParser.h"
#include "Utilities/RedBlackTree/RedBlackTree.h"

class Adapter {
public:
    static Adapter *getInstance() {
        static Adapter instance;
        return &instance;
    }
    ~Adapter() = default;
    Adapter(const Adapter &) = delete;
    void operator=(const Adapter &) = delete;

    void loadDefaultConfig();
    void jsonInitialize();
    void kinematicHandler(const POSE &pose, const VELOCITY &vel); // TODO will need to change for ROS
    void sonarHandler(const std::array<SONAR, 3> &sonar); // TODO will need to change for ROS
    void slamHandler();

    // TODO should be removed
    void aRH();
    void fsH();
    void tH();

//void auxilaryRoverHandler(const AuxilaryBeliefs &auxBeliefs);
//void featureSetHandler(const AuxilaryFeatureSets &auxFS);
//void transformationHandler(const TransformationPairs &transPairs);
    void publishFeatureSet(const std::array<FEATURE, FEATURE_LIMIT> &featureSet, const CLASSIFIER &classifer);
    void publishBelief(const POSE &pose, const float &confi);
    void publishTransformation(const POSE &trans, const std::string &rID);

private:
    Adapter() = default;

    std::unique_ptr<SharedMemory> sharedMemory;
    std::unique_ptr<ConfigParser> configParser;
    std::unique_ptr<Seif> seif;
    std::unique_ptr<Detection> detection;
    std::unique_ptr<RedBlackTree> localMap;
    std::unique_ptr<SYS_CONFIG_IN> systemConfig;

    std::string roverName;
    bool canReadLocation;
// TODO REMOVE used only in isolation (no ROS)
    const std::string published_name = "achilles";
};

#endif //MULTIAGENTSLAM_ADAPTER_H
