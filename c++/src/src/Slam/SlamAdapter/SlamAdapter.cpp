//
// Created by Mason U'Ren on 2019-02-13.
//

#include "SlamAdapter.h"

// TODO
void SlamAdapter::updateKinematics(const POSE &pose, const VELOCITY &vel) {

}

// TODO
void SlamAdapter::updateDetections(const std::array<SONAR, 3> &sonar) {

}

// TODO
void SlamAdapter::recordAuxilaryRoversBelief(const POSE &pose, const float confidence) {

}

// TODO
void SlamAdapter::slamUpdate() {

}

// TODO
void SlamAdapter::logAuxilaryFeatureSet(const std::array<FEATURE, 3> &features, const CLASSIFIER &classifier, const int publisher) {

}

void SlamAdapter::updateTransformationByRover(const POSE &transformation, const std::string &pairedRover) {
    float xTran = (*this->transformations)[pairedRover]->x_translation->filterValue(transformation.x);
    float yTran = (*this->transformations)[pairedRover]->y_translation->filterValue(transformation.y);
    float oTrain = (*this->transformations)[pairedRover]->orientation->filterValue(transformation.theta);
}

void SlamAdapter::addTransformation(std::string &roverName, Transformation *trans) {
    this->transformations->insert(std::make_pair(roverName, trans));
}