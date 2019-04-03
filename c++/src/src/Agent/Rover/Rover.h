#include <utility>

#include <utility>

//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_ROVER_H
#define C_ROVER_H

#define BOOST_ALLOW_DEPRECATED_HEADERS

#include <functional>
#include <SLAMConfigIn.h>
#include <RoverInterface.h>
#include <boost/uuid/uuid_generators.hpp>
#include "../../Slam/SEIF/Seif.h"
#include "../../Utilities/BinaryTree/RedBlackTree.h"
#include "../Detection/Detection.h"
#include "../Moments/Moments.h"

typedef std::function<void(float[3], int)> TransformationCallback;

class Rover : virtual public RoverInterface {
public:
    explicit Rover(std::string name = "DUMMY") :
        ID(boost::uuids::random_generator()()),
        name(std::move(name)),
        confidence(1.0),
        pose(new POSE {.x = 0, .y = 0, .theta = 0})
    {}

    explicit Rover(ROVER_CONFIG *roverConfig) :
        ID(boost::uuids::random_generator()()),
        name(roverConfig->name),
        confidence(1.0),
        pose(new POSE {.x = 0, .y = 0, .theta = 0}),
        vel(new VELOCITY {.linear = 0, .angular = 0})
//        ,
//        seif(new Seif(&roverConfig->seifConfig)),
//        detection(new Detection(&roverConfig->detectionConfig)),
//        localMap(new RedBlackTree(&roverConfig->localMapConfig))
    {}

    ~Rover() override = default;

    unsigned int getID() const override;
    std::string getName() const override;
    POSE *getCurrentPose() const override;
    VELOCITY *getVelocity() const override;
    float getConfidence() const override;

    // Allocate Memory, should only be used during initialization.
    void addSeif(Seif *seif);
    void addDetection(Detection *detection);
    void addLocalMap(RedBlackTree *localMap);

    void connectTransformationCallbacks(std::array<TransformationCallback, 2> &callbacks);
    void updatePoseVel(const POSE &pose, VELOCITY velocity);
    void updateMLIncidentRay(const std::array<SONAR, RANGE_SENSOR_COUNT> &sonar);
    void updateBelief(const POSE &pose, float confidence);
    void spareExtendedInformationFilter();
    void integrateLocalFS(std::array<FEATURE, MAX_FEATURES_IN_SET> features, float classifier);
    void integrateGlobalFS(std::array<FEATURE, MAX_FEATURES_IN_SET> features, float classifier, int publisher);

    // For testing purposes only!
    RedBlackTree *getLocalMap() {
        return &(*localMap);
    }
    Detection *getDetections() {
        return &(*detection);
    }

private:
    void setName(std::string name) override;
    void setCurrentPose(const POSE &belief) override;
    void setVelocity(VELOCITY velocity) override;
    void setConfidence(float confi) override;

    void integratePose(const POSE &pose);
    void integrateFilteredPose(const POSE &pose);
    void updateMoments();
    void updateMeans();
    void updateVariances();
    void tuneConfi();
    float *estimateMapTransformation(std::array<FEATURE, MAX_FEATURES_IN_SET> &features,
            std::array<FEATURE, MAX_FEATURES_IN_SET> &otherFeatures);
    float *mapTranslation(const std::array<float, 2> &fsOentroid, const std::array<float, 2> &otherOentroid);
    float mapOrientation(float fsOrientation, float otherOrientation);
    void shareTransformation(const std::array<float, 3> &transformation, int pairedRover);

    /**
     * Variables
     */
    boost::uuids::uuid ID;
    std::string name;
    float confidence;
    std::shared_ptr<POSE> pose;
    std::shared_ptr<VELOCITY> vel;
    std::shared_ptr<Seif> seif;
    std::shared_ptr<Detection> detection;
    std::shared_ptr<RedBlackTree> localMap;

    static bool writingPose;
    std::shared_ptr<std::array<TransformationCallback, 2>> callbacks;
};


#endif //C_ROVER_H
