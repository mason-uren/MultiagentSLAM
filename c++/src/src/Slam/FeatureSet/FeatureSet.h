//
// Created by Mason U'Ren on 2019-04-12.
//

#ifndef MULTIAGENTSLAM_FEATURESET_H
#define MULTIAGENTSLAM_FEATURESET_H

#include <iostream>
#include <array>
#include <tuple>

#include <SharedMemoryStructs.h>

#include "../../Utilities/Equations/Equations.h"

inline int idx(const int &i) { return i % FEATURE_LIMIT; };

class FeatureSet {
//    friend class Seif;
public:
    static FeatureSet *getInstance() {
        static FeatureSet instance;
        return &instance;
    }

    void addToSet(const FEATURE &feature, const POSE &rPose);
    bool readyToPublish();
    std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER> publishSet();

private:
    FeatureSet() :
            set(std::array<FEATURE, FEATURE_LIMIT>()),
            incidentOrient(std::array<float, FEATURE_LIMIT>()),
            classifier(CLASSIFIER{}),
            currFeatIdx(0)
    {}
    FeatureSet(const FeatureSet &) = delete;
    void operator=(const FeatureSet &) = delete;

    void incrPtr();
    void analyzeFeats();
    bool isSetFull();
    void fsArea();
    void fsOrientation();
    void fsSignature();

    std::array<FEATURE, FEATURE_LIMIT> set;
    std::array<float, FEATURE_LIMIT> incidentOrient;
    CLASSIFIER classifier; // TODO classifier may have to be reset each time a feature set is built
    int currFeatIdx;


};


#endif //MULTIAGENTSLAM_FEATURESET_H
