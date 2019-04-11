//
// Created by Mason U'Ren on 2019-02-13.
//

#include "Seif.h"

// IMPORTANT: should not be accessed/used outside of Seif
POSE Seif::rPose = POSE{};

void Seif::printRoverPose() {
    std::cout << "R-Pose : (" <<
        Seif::rPose.x << ", " <<
        Seif::rPose.y << ", " <<
        Seif::rPose.theta << ")" << std::endl;
}

// TODO
void Seif::connectFeatureCallback(FeatureCallback &callback) {

}

void Seif::motionUpdate(const VELOCITY &velocity) {
    if (this->printMatrices) {
        std::cout << "F_X:" << std::endl;
        this->F_X->print();
    }

    this->updateDeltaDel(velocity);
    this->updatePsi();
    this->updateLambda();
    this->updatePhi();
    this->updateKappa();
    this->updateOmegaBar();
    this->updateEpsilonBar();
    this->updateMuBar();
}

POSE Seif::stateEstimateUpdate() {
    Matrix<float> stateEstimate = *this->stateEstimate;
    this->integrateActiveFeatures();
    this->generateStateEstimate(&stateEstimate);
    rPose = {
        .x = Equations::getInstance()->isZero(this->stateEstimate->at(X)) ? 0 : this->stateEstimate->at(X),
        .y = Equations::getInstance()->isZero(this->stateEstimate->at(Y)) ? 0 : this->stateEstimate->at(Y),
        .theta = Equations::getInstance()->wrapTheta(this->stateEstimate->at(THETA))
    };
    return rPose;
}

// Must always run regardless of valid detection. Found features still need to
// updated with new rover estimate.
void Seif::measurementUpdate(const RAY &incidentRay) {
    FEATURE feature{};
    if (this->isNewFeature(incidentRay)) {
        this->deriveFeature(feature, incidentRay);
        if (!this->hasBeenObserved(feature.correspondence)) {
            this->addFeature(feature);
            this->organizeFeatures();
        }
    }
    Matrix<float> vecSum(N);
    Matrix<float> matrixSum(N, N);
    for (unsigned long featIdx = 0; featIdx < this->featuresFound; featIdx++) {
        feature = (*this->recordedFeatures)[featIdx];
        this->updateDeltaPos(feature.pose);
        this->update_q();
        this->updateZHat(feature.correspondence);
        this->updateH(featIdx);
        this->infoVecSummation(feature);
        this->infoMatrixSummation();
    }
}

void Seif::sparsification() {
    Matrix<float> infoMat = *this->informationMatrix;
    this->updateInformationMatrix();
    this->updateInformationVector(&infoMat);
}

void Seif::updateDeltaDel(const VELOCITY &velocity) {
    float ratio;
    float thetaPrior = rPose.theta;
    float theta = (float) (velocity.angular * ROS_INTERVAL);
    if (velocity.angular != 0) {
        ratio = velocity.linear / velocity.angular;
        this->delta->at(X) = (-1) * ratio * sin(thetaPrior) + ratio * sin(thetaPrior + theta);
        this->delta->at(Y) = ratio * cos(thetaPrior) - ratio * cos(thetaPrior + theta);

        this->del->at(X, 2) = ratio * cos(thetaPrior) - ratio * cos(thetaPrior + theta);
        this->del->at(Y, 2) = ratio * sin(thetaPrior) - ratio * sin(thetaPrior + theta);
    }
    else {
        ratio = velocity.linear;
        this->delta->at(X) = ratio * sin(thetaPrior);
        this->delta->at(Y) = ratio * cos(thetaPrior);

        this->del->at(X, 2) = (-1) * ratio * cos(thetaPrior);
        this->del->at(Y, 2) = (-1) * ratio * sin(thetaPrior);
    }
    this->delta->at(THETA) = theta;

    if (this->printMatrices) {
        std::cout << "Delta: " << std::endl;
        this->delta->print();
        std::cout << "Del: " << std::endl;
        this->del->print();
    }
}

void Seif::updatePsi() {
    Matrix<float> idMat(this->del->numRows(), this->del->numRows()); idMat.identity();
    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    Matrix<float> inverse = (idMat + *this->del); inverse.invert();
    *this->psi = fx_T * (inverse - idMat) * *this->F_X;

    if (this->printMatrices) {
        std::cout << "PSI:" << std::endl;
        this->psi->print();
    }
}

void Seif::updateLambda() {
    Matrix<float> psi_T = *this->psi; psi_T.transpose();
    *this->lambda = psi_T * *this->informationMatrix +
            *this->informationMatrix * *this->phi +
            psi_T * *this->informationMatrix * *this->phi;

    if (this->printMatrices) {
        std::cout << "Lambda:" << std::endl;
        this->lambda->print();
    }
}

void Seif::updatePhi() {
    *this->phi = *this->informationMatrix + *this->lambda;

    if (this->printMatrices) {
        std::cout << "PHI:" << std::endl;
        this->phi->print();
    }
}

void Seif::updateKappa() {
    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    Matrix<float> R_inv = *this->motionCov; R_inv.invert();
    Matrix<float> inverse = (R_inv + *this->F_X * *this->phi * fx_T); inverse.invert();
    *this->kappa = *this->phi * fx_T * inverse * *this->F_X * *this->phi;

    if (this->printMatrices) {
        std::cout << "Kappa:" << std::endl;
        this->kappa->print();
    }
}

void Seif::updateOmegaBar() {
    *this->phi -= *this->kappa;
    *this->informationMatrix = *this->phi;

    if (this->printMatrices) {
        std::cout << "Info Mat:" << std::endl;
        this->informationMatrix->print();
    }
}

void Seif::updateEpsilonBar() {
    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    *this->informationVector +=
            ((*this->lambda - *this->kappa) * *this->stateEstimate +
            (*this->informationMatrix * fx_T * *this->delta));

    if (this->printMatrices) {
        std::cout << "Info Vec: " << std::endl;
        this->informationVector->print();
    }
}

void Seif::updateMuBar() {
    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    *this->stateEstimate += fx_T * *this->delta;

    if (this->printMatrices) {
        std::cout << "State estimate: " << std::endl;
        this->stateEstimate->print();
    }
}

void Seif::integrateActiveFeatures() {
    unsigned long startIdx;
    this->F_I->zeroMatrix();
    Matrix<float> resVec(this->informationVector->numRows());

    for (long i = 0; i < std::fmin(this->featuresFound, this->maxActiveFeatures); i++) {
        FEATURE *feature = &((*this->activeFeatures)[i]);
        if (!feature) {
            break;
        }

        startIdx = featIdx(feature->idx);
        this->F_I->at(X, startIdx) = 1;
        this->F_I->at(Y, startIdx + Y) = 1;

        Matrix<float> fi_T = *this->F_I; fi_T.transpose();
        Matrix<float> inverse = (*this->F_I * *this->informationMatrix * fi_T); inverse.invert();
        resVec = inverse * *this->F_I *
                (*this->informationVector - *this->informationMatrix * *this->stateEstimate +
                *this->informationMatrix * fi_T * *this->F_I * *this->stateEstimate);
        this->stateEstimate->at(startIdx) = resVec.at(X);
        this->stateEstimate->at(startIdx + Y) = resVec.at(Y);

        if (this->printMatrices) {
            std::cout << "State Estimate (Active):" << std::endl;
            this->stateEstimate->print();
        }
    }
}

void Seif::generateStateEstimate(const Matrix<float> *stateEstimate) {
    Matrix<float> resVec(this->informationVector->numRows());
    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    Matrix<float> inverse = (*this->F_X * *this->informationMatrix * fx_T); inverse.invert();
    resVec = inverse * *this->F_X *
             (*this->informationVector - *this->informationMatrix * *stateEstimate +
              *this->informationMatrix * fx_T * *this->F_X * *stateEstimate);
    this->stateEstimate->at(X) = resVec.at(X);
    this->stateEstimate->at(Y) = resVec.at(Y);
    this->stateEstimate->at(THETA) = resVec.at(THETA);

    if (this->printMatrices) {
        std::cout << "State Estimate (All):" << std::endl;
        this->stateEstimate->print();
    }
}

bool Seif::isNewFeature(const RAY &incidentRay) {
    return incidentRay.range != -MAXFLOAT && incidentRay.angle != -MAXFLOAT;
}

void Seif::deriveFeature(FEATURE &feature, const RAY &incidentRay) {
    std::array<float, 2> xyCoords = Equations::getInstance()->originToPoint(
            incidentRay,
            {rPose.x, rPose.y, rPose.theta},
            true); // true

    feature.incidentRay = incidentRay;
    feature.pose = {.x = xyCoords[X], .y = xyCoords[Y]};
    feature.correspondence = Equations::getInstance()->normalizeValue(
            Equations::getInstance()->cantor(xyCoords[X], xyCoords[Y]),
            0, this->maxCorrespondence
    );
}

/**
 * @fn hasBeenObserved
 * @brief Utilizes a binary search to check for if feature has already been obeserved.
 *
 * Observation is confirmed/denied depending on whether the distance between two features
 * falls below #minFeatureDist.
 * @param correspondence - the associated feature identifier
 * @return Was the currently viewed feature previously observed.
 */
bool Seif::hasBeenObserved(const float &correspondence) {
    u_long front = 0;
    u_long back = this->featuresFound ? this->featuresFound - 1 : 0;

    if (!this->featuresFound) {
        return false;
    }
    if (this->featuresFound < 2) {
        return EQUIV == comparison(correspondence, (*this->recordedFeatures)[front].correspondence);
    }

    bool finished = false;
    do {
        unsigned long position = (u_long) ((front && back) ? floor((front + back) / 2) : 0);
        relation section = comparison(correspondence, (*this->recordedFeatures)[position].correspondence);
        switch (section) {
            case EQUIV:
                return true;
            case LOWER:
                if (!position) {
                    finished = true;
                }
                back = --position;
                break;
            case HIGHER:
                front = ++position;
                break;
            default:
                std::cout << "Error: bad feature identifier comparison <" << section << ">" << std::endl;
                exit(EXIT_FAILURE);
        }
    } while (front <= back && !finished);

    return false;
}

void Seif::addFeature(FEATURE &feature) {
    feature.idx = this->nextFeatureIndex();
    u_long idx = feature.idx;
    if (this->isActiveFull()) {
        *this->toDeactivate = (*this->activeFeatures)[idx = 0]; // Keep the furthest away feature in the initial position;
    }
    (*this->activeFeatures)[idx] = feature;
    (*this->recordedFeatures)[this->nextFeatureIndex()++] = feature;
}

u_long &Seif::nextFeatureIndex() {
    if (this->featuresFound < this->maxFeatures) {
        return this->featuresFound;
    }
    perror("Maximum number of features observed. Consider allowing more observed features : 'maxFeatures'.");
    exit(EXIT_FAILURE);
}

void Seif::organizeFeatures() {
    if (this->featuresFound > 1) {
        auto endPtr_active = (this->isActiveFull() ?
                              this->activeFeatures->end() :
                              this->activeFeatures->begin() + this->featuresFound);
        std::sort(this->activeFeatures->begin(), endPtr_active, distanceSort);

        auto endPtr_all = this->recordedFeatures->begin() + this->featuresFound;
        std::sort(this->recordedFeatures->begin(), endPtr_all, correspondenceSort);
    }
}

relation Seif::comparison(const float &identifier, const float &otherID) {
    return (abs(identifier - otherID) < minFeatureDist) ? EQUIV : ((identifier < otherID) ? LOWER : HIGHER);
}

bool Seif::isActiveFull() {
    return this->featuresFound >= this->maxActiveFeatures;
}

void Seif::updateDeltaPos(const POSE &featPose) {
    this->deltaPosition->at(X) = featPose.x - rPose.x;
    this->deltaPosition->at(Y) = featPose.y - rPose.y;
}

void Seif::update_q() {
    Matrix<float> dp_T = *this->deltaPosition; dp_T.transpose();
    this->q = (dp_T * *this->deltaPosition).at(0);
}

void Seif::updateZHat(const float &correspondence) {
    this->zHat->at(RANGE) = sqrt(this->q);
    this->zHat->at(ANGLE) = atan2(this->deltaPosition->at(Y), this->deltaPosition->at(X)) - rPose.theta;
    this->zHat->at(CORRESPONDENCE) = correspondence;
}

void Seif::updateH(const unsigned long &idx) {
    this->H->at(X, X) = sqrt(this->q) * this->deltaPosition->at(X);
    this->H->at(X, Y) = (-1) * sqrt(this->q) * this->deltaPosition->at(Y);
    this->H->at(Y, X) = this->deltaPosition->at(Y);
    this->H->at(Y, Y) = this->deltaPosition->at(X);
    this->H->at(Y, 2) = -1;

    unsigned long startIdx = featIdx(idx);

    this->H->at(X, startIdx + X) = (-1) * sqrt(this->q) * this->deltaPosition->at(X);
    this->H->at(X, startIdx + Y) = sqrt(this->q) * this->deltaPosition->at(Y);
    this->H->at(Y, startIdx + X) = (-1) * this->deltaPosition->at(Y);
    this->H->at(Y, startIdx + Y) = (-1) * this->deltaPosition->at(X);
    this->H->at(2, startIdx + 2) = 1;

    *this->H *= (1 / this->q);
}

void Seif::infoVecSummation(const FEATURE &feature) {
    Matrix<float> z_i{{feature.incidentRay.range}, {feature.incidentRay.angle}, {feature.correspondence}};
    Matrix<float> H_T = *this->H; H_T.transpose();
    Matrix<float> Q_inv = *this->measurementCov; Q_inv.invert();
    *this->informationVector += H_T * Q_inv * (z_i - *this->zHat - *this->H * *this->stateEstimate);
}

void Seif::infoMatrixSummation() {
    Matrix<float> H_T = *this->H; H_T.transpose();
    Matrix<float> Q_inv = *this->measurementCov; Q_inv.invert();
    *this->informationMatrix += H_T * Q_inv * *this->H;
}



void Seif::updateInformationMatrix() {
    Matrix<float> m0_T = this->defineProjection(&(*this->toDeactivate), false); m0_T.transpose();
    Matrix<float> xm0_T = this->defineProjection(&(*this->toDeactivate)); xm0_T.transpose();
    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    if (this->toDeactivate->correspondence >= 0) {
        this->makeInactive(&(*this->toDeactivate));
    }

    *this->informationMatrix -= this->resolveProjection(&m0_T) + this->resolveProjection(&xm0_T) - this->resolveProjection(&fx_T);
}

void Seif::updateInformationVector(const Matrix<float> *prevInfoMat) {
    *this->informationVector += (*this->informationMatrix - *prevInfoMat) * *this->stateEstimate;
}

Matrix<float> Seif::resolveProjection(const Matrix<float> *projection) {
    Matrix<float> p_T = *projection; p_T.transpose();
    Matrix<float> inverse = p_T * *this->informationMatrix * *projection; inverse.invert();
    return *this->informationMatrix * *projection * inverse * p_T * *this->informationMatrix;
}

Matrix<float> Seif::defineProjection(const FEATURE *feat, const bool &includePose) {
    Matrix<float> projection(this->F_X->numRows(), this->F_X->numCols());
    if (includePose) {
        projection.at(X, X) = 1;
        projection.at(Y, Y) = 1;
        projection.at(THETA, THETA) = 1;
    }
    if (feat->correspondence >= 0) {
        unsigned long startIdx = featIdx(feat->idx);
        projection.at(X, startIdx) = 1;
        projection.at(Y, startIdx + Y) = 1;
        projection.at(CORRESPONDENCE, startIdx + CORRESPONDENCE) = 1;
    }

    return projection;
}

void Seif::makeInactive(FEATURE *toDeact) {
    toDeact->correspondence = -MAXFLOAT;
}

/**
 * @fn correspondenceSort
 * @brief sort #FEATUREs based on their correspondence values.
 *
 * Sorts from lowest to highest.
 * @param feat reference to a feature
 * @param other reference to a different feaure
 * @return is the correspondence of #feat lower than #other.
 */
bool Seif::correspondenceSort(const FEATURE &feat, const FEATURE &other) {
    return feat.correspondence < other.correspondence;;
}

/**
 * @fn distanceSort
 * @brief sort #FEATUREs based on their relationship to the current rover position.
 *
 * Sorts from highest to lowest.
 * @param featA reference to a feature
 * @param featB reference to a feature
 * @return is the relative distance between rover and #featA larger than that with #featB.
 */
bool Seif::distanceSort(const FEATURE &featA, const FEATURE &featB) {
    float distA = Equations::getInstance()->distBetweenPts(featA.pose, rPose);
    float distB = Equations::getInstance()->distBetweenPts(featB.pose, rPose);
    return distA > distB;
}

// TODO
void Seif::logFeature(const FEATURE &feature, const std::array<float, 3> roverPose) {

}

POSE Seif::getRoverPose() {
    return rPose;
}
