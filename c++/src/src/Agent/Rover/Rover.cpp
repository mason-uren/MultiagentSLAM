//
// Created by Mason U'Ren on 2019-02-13.
//

#include "Rover.h"
unsigned int Rover::getID() const {
    return static_cast<unsigned int>(*this->ID.data);
}

std::string Rover::getName() const {
    return this->name;
}

VELOCITY * Rover::getVelocity() const {
    return &(*this->vel);
}

POSE * Rover::getCurrentPose() const {
    return &(*this->pose);
}

float Rover::getConfidence() const {
    return this->confidence;
}

void Rover::addSeif(Seif *seif) {
    if (!this->seif) {
        this->seif = std::shared_ptr<Seif>(seif);
    }
}

void Rover::addDetection(Detection *detection) {
    if (!this->detection) {
        this->detection = std::shared_ptr<Detection>(detection);
    }
}

void Rover::addLocalMap(RedBlackTree *localMap) {
    if (!this->localMap) {
        this->localMap = std::shared_ptr<RedBlackTree>(localMap);
    }
}

// TODO
void Rover::connectTransformationCallbacks(std::array<TransformationCallback, 2> &callbacks) {

}

// TODO
void Rover::updatePoseVel(const POSE &pose, const VELOCITY velocity) {

}

// TODO
void Rover::updateMLIncidentRay(const std::array<SONAR, 3> &sonar) {

}

// TODO
void Rover::updateBelief(const POSE &pose, float confidence) {

}

// TODO
void Rover::spareExtendedInformationFilter() {

}

// TODO
void Rover::integrateLocalFS(const std::array<FEATURE, 3> features, float classifier) {

}

// TODO
void Rover::integrateGlobalFS(const std::array<FEATURE, 3> features, float classifier, int publisher) {

}

void Rover::setName(std::string name) {
    this->name = name;
}

void Rover::setVelocity(VELOCITY velocity) {
    *this->vel = velocity;
}

void Rover::setCurrentPose(const POSE &belief) {
    *this->pose = belief;
}

void Rover::setConfidence(float confi) {
    this->confidence = confi;
}

// TODO
void Rover::integratePose(const POSE &pose) {

}

// TODO
void Rover::integrateFilteredPose(const POSE &pose) {

}

// TODO
void Rover::updateMoments() {

}

// TODO
void Rover::updateMeans() {

}

// TODO
void Rover::updateVariances() {

}

// TODO
void Rover::tuneConfi() {

}

// TODO
float *Rover::estimateMapTransformation(std::array<FEATURE, 3> &features, std::array<FEATURE, 3> &otherFeatures) {
    return nullptr;
}

// TODO
float *Rover::mapTranslation(const std::array<float, 2> &fsOentroid, const std::array<float, 2> &otherOentroid) {
    return nullptr;
}

// TODO
float Rover::mapOrientation(float fsOrientation, float otherOrientation) {
    return 0;
}

// TODO
void Rover::shareTransformation(const std::array<float, 3> &transformation, int pairedRover) {

}
