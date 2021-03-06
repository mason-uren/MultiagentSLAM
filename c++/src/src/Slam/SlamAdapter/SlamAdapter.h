//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SLAMADAPTER_H
#define C_SLAMADAPTER_H

#include <SharedMemoryStructs.h>
#include <SLAMConfigIn.h>

#include "../../Agent/Rover/Rover.h"
#include "../ActiveRovers/ActiveRovers.h"
#include "../Transformation/Transformation.h"

using std::string;

class SlamAdapter {
public:
    static SlamAdapter *getInstance() {
        static SlamAdapter instance;
        return &instance;
    }

    void updateKinematics(const string &rName, const POSE &pose, const VELOCITY &vel);
    void updateDetections(const string &rName, const std::array<SONAR, 3> &sonar);
    void recordAuxilaryRoversBelief(const string &rName, const POSE &pose, float confidence);

    void slamUpdate(const string &rName);
    void logAuxilaryFeatureSet(const string &rName, const std::array<FEATURE, 3> &features, const CLASSIFIER &classifier, const string &publisher);
    void updateTransformationByRover(const POSE &transformation, const std::string &pairedRover);
    void addTransformation(std::string &roverName, Transformation *trans);

    // FOR TESTING PURPOSES
    std::unordered_map<std::string, Transformation *> *getTransformations() {
        return transformations.get();
    }

private:
    SlamAdapter() :
            rover(std::shared_ptr<Rover>(new Rover())),
            transformations(new std::unordered_map<std::string, Transformation *>())
            {}

    SlamAdapter(SlamAdapter const&) = delete;
    void operator=(SlamAdapter const&) = delete;

    /**
     * Variables
     */
    std::shared_ptr<Rover> rover;
    std::shared_ptr<std::unordered_map<std::string, Transformation *>> transformations;
};


#endif //C_SLAMADAPTER_H
