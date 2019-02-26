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

class SlamAdapter {
public:
    static SlamAdapter *getInstance() {
        static SlamAdapter instance;
        return &instance;
    }

    void updateKinematics(const POSE &pose, const VELOCITY &vel);
    void updateDetections(const std::array<SONAR, 3> &sonar);
    void recordAuxilaryRoversBelief(const POSE &pose, float confidence);

    void slamUpdate();
    void logAuxilaryFeatureSet(const std::array<FEATURE, 3> &features, const CLASSIFIER &classifier, int publisher);
    void updateTransformationByRover(const std::array<float, 3> &transformation, std::string &pairedRover);
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

    SlamAdapter(SlamAdapter const&);
    void operator=(SlamAdapter const&);

    /**
     * Variables
     */
    std::shared_ptr<Rover> rover;
    std::shared_ptr<std::unordered_map<std::string, Transformation *>> transformations;
};


#endif //C_SLAMADAPTER_H
