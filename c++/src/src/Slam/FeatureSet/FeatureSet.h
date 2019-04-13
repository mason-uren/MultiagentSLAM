//
// Created by Mason U'Ren on 2019-04-12.
//

#ifndef MULTIAGENTSLAM_FEATURESET_H
#define MULTIAGENTSLAM_FEATURESET_H

#include <iostream>
#include <array>
#include <SharedMemoryStructs.h>

#include "../../Utilities/Equations/Equations.h"

#define FEATURE_LIMIT 3
#define SIGNATURE_MAX 100

typedef std::function<void(FEATURE[FEATURE_LIMIT], float[2])> FeatureSetCallback;

inline int idx(const int &i) { return i % FEATURE_LIMIT; };

class FeatureSet {
public:
    FeatureSet() :
        set(std::array<FEATURE, FEATURE_LIMIT>()),
        incidentOrient(std::array<float, FEATURE_LIMIT>()),
        classifier(CLASSIFIER{}),
        currFeatIdx(0)
    {}
    ~FeatureSet() = default;

    void connectFSCallbacks(const std::array<FeatureSetCallback, 2> &calls);
    void addToSet(const FEATURE &feature, const POSE &rPose);
    void publishSet();

private:
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

    std::shared_ptr<std::array<FeatureSetCallback, 2>> callbacks;

};


#endif //MULTIAGENTSLAM_FEATURESET_H
