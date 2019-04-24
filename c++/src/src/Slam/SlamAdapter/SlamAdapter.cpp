//
// Created by Mason U'Ren on 2019-02-13.
//

#include "SlamAdapter.h"

void SlamAdapter::updateKinematics(const string &rName, const POSE &pose, const VELOCITY &vel) {
    *rover = ActiveRovers::getInstance()->getRoverByName(rName);
    rover->updatePoseVel(pose, vel);
}

void SlamAdapter::updateDetections(const string &rName, const std::array<SONAR, 3> &sonar) {
    *rover = ActiveRovers::getInstance()->getRoverByName(rName);
    rover->updateMLIncidentRay(sonar);
}

void SlamAdapter::recordAuxilaryRoversBelief(const string &rName, const POSE &pose, const float confidence) {
    *rover = ActiveRovers::getInstance()->getRoverByName(rName);
    rover->updateBelief(pose, confidence);
}

void SlamAdapter::slamUpdate(const string &rName) {
    *rover = ActiveRovers::getInstance()->getRoverByName(rName);
    rover->spareExtendedInformationFilter();
}

void SlamAdapter::logAuxilaryFeatureSet(const string &rName, const std::array<FEATURE, 3> &features, const CLASSIFIER &classifier, const string &publisher) {
    *rover = ActiveRovers::getInstance()->getRoverByName(rName);
    rover->integrateGlobalFS(features, classifier, publisher);
}

void SlamAdapter::updateTransformationByRover(const POSE &transformation, const std::string &pairedRover) {
    float xTran = (*this->transformations)[pairedRover]->x_translation->filterValue(transformation.x);
    float yTran = (*this->transformations)[pairedRover]->y_translation->filterValue(transformation.y);
    float oTrain = (*this->transformations)[pairedRover]->orientation->filterValue(transformation.theta);
}

void SlamAdapter::addTransformation(std::string &roverName, Transformation *trans) {
    this->transformations->insert(std::make_pair(roverName, trans));
}
