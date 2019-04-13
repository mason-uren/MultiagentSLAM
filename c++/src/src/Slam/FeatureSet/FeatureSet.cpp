//
// Created by Mason U'Ren on 2019-04-12.
//

#include "FeatureSet.h"

void FeatureSet::connectFSCallbacks(const std::array<FeatureSetCallback, 2> &calls) {

}

void FeatureSet::addToSet(const FEATURE &feature, const POSE &rPose) {
    set[this->currFeatIdx] = feature;
    incidentOrient[this->currFeatIdx] = rPose.theta;
    this->incrPtr();
    if (this->isSetFull()) {
        this->analyzeFeats();
        this->publishSet();
    }
}

void FeatureSet::publishSet() {
    std::cout << "Publishing Set" << std::endl;
    std::cout << "Area : " << classifier.area << std::endl;
    std::cout << "Orientation : " << classifier.orientation << std::endl;
    std::cout << "Classifier : " << classifier.signature << std::endl;
}

void FeatureSet::incrPtr() {
    ++this->currFeatIdx %= FEATURE_LIMIT;
}

void FeatureSet::analyzeFeats() {
    this->fsArea();
    this->fsOrientation();
    this->fsSignature();
}

bool FeatureSet::isSetFull() {
    static bool run = false;
    if (!run) {
        run = true;
    }
    return !static_cast<bool>(this->currFeatIdx % FEATURE_LIMIT);
}

void FeatureSet::fsArea() {
    std::array<float, FEATURE_LIMIT> legs {
        Equations::getInstance()->distBetweenPts(set[0].pose, set[1].pose),
        Equations::getInstance()->distBetweenPts(set[0].pose, set[2].pose),
        Equations::getInstance()->distBetweenPts(set[1].pose, set[2].pose)
    };
    float area = (legs[0] + legs[1] + legs[2]) / 2;
    classifier.area =  sqrtf(area * (area - legs[0]) * (area - legs[1]) * (area - legs[2]));
}

void FeatureSet::fsOrientation() {
    for (int i = 0; i < FEATURE_LIMIT; i++) {
        classifier.orientation += Equations::getInstance()->wrapTheta(set[i].incidentRay.angle + incidentOrient[i]);
    }
}

void FeatureSet::fsSignature() {
    classifier.signature =  Equations::getInstance()->normalizeValue(
            Equations::getInstance()->cantor(classifier.area, classifier.orientation),
            0, SIGNATURE_MAX
    );
}
