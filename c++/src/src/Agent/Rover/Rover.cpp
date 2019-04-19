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
    this->seif->motionUpdate(*this->vel);
    this->seif->stateEstimateUpdate();
    this->seif->measurementUpdate(*this->detection->getIncidentRay());
    this->seif->sparsification();
}

void Rover::integrateLocalFS(const std::array<FEATURE, FEATURE_LIMIT> &features, const CLASSIFIER &classifier) {
    u_long idx;
    if(!this->localMap->findMLClassifier(classifier, idx)) {
        this->localMap->addToTree(classifier, features);
    }
}

void Rover::integrateGlobalFS(const std::array<FEATURE, FEATURE_LIMIT> &foundFeat, const CLASSIFIER &classifier, const string &publisher) {
    u_long idx;
    if (this->localMap->findMLClassifier(classifier, idx)) {
        auto features{std::array<FEATURE, FEATURE_LIMIT>()};
        this->localMap->getFeaturesFromNode(features, idx);
        transformation = tuple<POSE, string>(this->estimateMapTransformation(features, foundFeat), publisher);
    }
}

bool Rover::readyToPublish() {
    if (this->canPublish) {
        this->canPublish = false;
        return true;
    }
    return false;
}

tuple<POSE, string> Rover::publish() {
    return transformation;
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

POSE Rover::estimateMapTransformation( const array<FEATURE, FEATURE_LIMIT> &fs_1, const array<FEATURE, FEATURE_LIMIT> &fs_2) {
    auto cent_1(Equations::getInstance()->centroid({
        fs_1[0].pose,
        fs_1[1].pose,
        fs_1[2].pose
    }));
    auto cent_2(Equations::getInstance()->centroid({
         fs_2[0].pose,
         fs_2[1].pose,
         fs_2[2].pose
    }));
    auto xyTrans(this->mapTranslation(cent_1, cent_2));
    auto orient(this->mapOrientation(fs_1[0].pose.theta, fs_2[0].pose.theta));
    return POSE{xyTrans.x, xyTrans.y, orient};
}

LOCATION Rover::mapTranslation(const LOCATION &fsCentroid, const LOCATION &otherCentroid) {
    return LOCATION{.x = fsCentroid.x - otherCentroid.x, .y = fsCentroid.y - otherCentroid.y};
}

float Rover::mapOrientation(const float &fsOrientation, const float &otherOrientation) {
    return Equations::getInstance()->wrapTheta(fsOrientation - otherOrientation);
}
