//
// Created by Mason U'Ren on 2019-02-13.
//

#include "Seif.h"

// IMPORTANT: should not be accessed/used outside of Seif
static POSE rPose = POSE{};

// TODO
void Seif::connectFeatureCallback(FeatureCallback &callback) {

}

void Seif::motionUpdate(const VELOCITY &velocity) {
    if (this->printMatrices) {
        std::cout << "F_X:" << std::endl;
        this->F_X->print();
    }

//    this->initializeMean(velocity);

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
    this->resolveActiveFeats();
    this->resolveAllFeats(&stateEstimate);
    rPose = {.x = this->stateEstimate->at(X),
             .y = this->stateEstimate->at(Y),
             .theta = this->stateEstimate->at(THETA)};
    return rPose;
}

// Is not concerned with whether RAY is below threshold. Thresholding, should
// be managed by Detection and a ray only passed to when a "valid" detection is made.
void Seif::measurementUpdate(const RAY &incidentRay) {
    FEATURE feature{};
    this->remodel(feature, incidentRay);
//    this->updateQ(incidentRay, feature.correspondence);

    if (!this->hasBeenObserved(feature.correspondence)) {
        this->addFeature(feature);
        this->organizeFeatures();
    }

    // Loop through all observed features (active and inactive)
    for (unsigned long featIdx = 0; featIdx < this->featuresFound; featIdx++) {
        feature = (*this->recordedFeatures)[featIdx];
        this->updateDeltaPos(feature.pose);
        this->update_q();
        this->updateZHat(feature.correspondence);
        this->updateH(featIdx);
        this->updateEpsilon(feature);
        this->updateOmega();
    }
}

void Seif::sparsification() {
    Matrix<float> infoMat = *this->informationMatrix;
    this->updateInformationMatrix();
    this->updateInformationVector(&infoMat);
}

void Seif::initializeMean(const VELOCITY &velocity) {
    static bool isSet = false;
    if (!isSet) {
        isSet = true;

        float linAngRatio = (velocity.angular != 0) ? velocity.linear / velocity.angular : velocity.linear;
        float thetaPrior = this->stateEstimate->at(THETA);

        this->stateEstimate->at(X) = (float) ((-1) * linAngRatio * sin(thetaPrior) + linAngRatio * sin(thetaPrior + (velocity.angular * ROS_INTERVAL)));
        this->stateEstimate->at(Y) = (float) (linAngRatio * cos(thetaPrior) - linAngRatio * cos(thetaPrior + (velocity.angular * ROS_INTERVAL)));
        this->stateEstimate->at(THETA) = (float) (velocity.angular * ROS_INTERVAL);
    }
}

void Seif::updateDeltaDel(const VELOCITY &velocity) {
    float ratio;
    float thetaPrior = this->stateEstimate->at(THETA);;
    if (velocity.angular != 0) {
        ratio = velocity.linear / velocity.angular;
        this->delta->at(X) = (float) (ratio * (sin(thetaPrior + (velocity.angular * ROS_INTERVAL) - sin(thetaPrior))));
        this->delta->at(Y) = (float) (ratio * (cos(thetaPrior) - cos(thetaPrior + (velocity.angular * ROS_INTERVAL))));

        this->del->at(X, 2) = (float) (ratio * (cos(thetaPrior) - cos(thetaPrior + (velocity.angular * ROS_INTERVAL))));
        this->del->at(Y, 2) = (float) (ratio * (sin(thetaPrior) - sin(thetaPrior + (velocity.angular * ROS_INTERVAL))));
    }
    else {
        ratio = velocity.linear;
        this->delta->at(X) = (float) (ratio * (-1) *sin(thetaPrior + (velocity.angular * ROS_INTERVAL)));
        this->delta->at(Y) = (float) (ratio * cos(thetaPrior + (velocity.angular * ROS_INTERVAL)));

        this->del->at(X, 2) = (float) (ratio * cos(thetaPrior + (velocity.angular * ROS_INTERVAL)));
        this->del->at(Y, 2) = (float) (ratio * sin(thetaPrior + (velocity.angular * ROS_INTERVAL)));
    }
    this->delta->at(THETA) = (float) (velocity.angular * ROS_INTERVAL);

    if (this->printMatrices) {
        std::cout << "Delta: " << std::endl;
        this->delta->print();
        std::cout << "Del: " << std::endl;
        this->del->print();
    }
}

void Seif::updatePsi() {
    Matrix<float> idMat = this->identity(this->del->numRows());
    Matrix<float> temp = idMat;

    MatrixManipulator::getInstance()->add(&temp, &(*this->del));
    temp.invert();
    MatrixManipulator::getInstance()->subtract(&temp, &idMat);
    *this->psi = MatrixManipulator::getInstance()->quadratic(&(*this->F_X), &temp);

    if (this->printMatrices) {
        std::cout << "PSI:" << std::endl;
        this->psi->print();
    }
}

void Seif::updateLambda() {
    Matrix<float> psi_T = *this->psi; psi_T.transpose();
    Matrix<float> prodMat = MatrixManipulator::getInstance()->multiply(&psi_T, &(*this->informationMatrix));
    Matrix<float> otherProdMat = MatrixManipulator::getInstance()->multiply(&(*this->informationMatrix), &(*this->psi));
    Matrix<float> temp = MatrixManipulator::getInstance()->quadratic(&(*this->psi), &(*this->informationMatrix));
    MatrixManipulator::getInstance()->add(&prodMat, &otherProdMat);
    MatrixManipulator::getInstance()->add(&prodMat, &temp);
    *this->lambda = prodMat;

    if (this->printMatrices) {
        std::cout << "Lambda:" << std::endl;
        this->lambda->print();
    }
}

void Seif::updatePhi() {
    Matrix<float> infoMat = *this->informationMatrix;
    MatrixManipulator::getInstance()->add(&infoMat, &(*this->lambda));
    *this->phi = infoMat;

    if (this->printMatrices) {
        std::cout << "PHI:" << std::endl;
        this->phi->print();
    }
}

void Seif::updateKappa() {
    Matrix<float> quadMat = MatrixManipulator::getInstance()->quadratic(&(*this->F_X), &(*this->phi));

    Matrix<float> R = *this->motionCov;
    R.invert();

    MatrixManipulator::getInstance()->add(&R, &quadMat);
    R.invert();

    Matrix<float> quadTemp = MatrixManipulator::getInstance()->quadratic(&(*this->F_X), &R);
    Matrix<float> prodMat = MatrixManipulator::getInstance()->multiply(&(*this->phi), &quadTemp);
    *this->kappa = MatrixManipulator::getInstance()->multiply(&prodMat, &(*this->phi));

    if (this->printMatrices) {
        std::cout << "Kappa:" << std::endl;
        this->kappa->print();
    }
}

void Seif::updateOmegaBar() {
    Matrix<float> temp = *this->phi;
    MatrixManipulator::getInstance()->subtract(&temp, &(*this->kappa));
    *this->informationMatrix = temp;

    if (this->printMatrices) {
        std::cout << "Info Mat:" << std::endl;
        this->informationMatrix->print();
    }
}

void Seif::updateEpsilonBar() {
    Matrix<float> temp = *this->lambda;
    MatrixManipulator::getInstance()->subtract(&temp, &(*this->kappa));
    temp = MatrixManipulator::getInstance()->multiply(&temp, &(*this->stateEstimate));

    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    Matrix<float> prodMat = MatrixManipulator::getInstance()->multiply(&(*this->informationMatrix), &fx_T);
    Matrix<float> otherProdMat = MatrixManipulator::getInstance()->multiply(&prodMat, &(*this->delta));
    MatrixManipulator::getInstance()->add(&(*this->informationVector), &temp);
    MatrixManipulator::getInstance()->add(&(*this->informationVector), &otherProdMat);

    if (this->printMatrices) {
        std::cout << "Info Vec: " << std::endl;
        this->informationVector->print();
    }
}

void Seif::updateMuBar() {
    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    Matrix<float> prodMat = MatrixManipulator::getInstance()->multiply(&fx_T, &(*this->delta));
    MatrixManipulator::getInstance()->add(&(*this->stateEstimate), &prodMat);

    if (this->printMatrices) {
        std::cout << "State estimate: " << std::endl;
        this->stateEstimate->print();
    }
}

//Matrix<float> Seif::motionCovError() {
//    Matrix<float> cov(ELEMENT_SIZE, ELEMENT_SIZE);
////    this->covMotionError->at(0, 0) = (*Moments::getInstance()->getMotion()).variances[X]->getFilteredVariance();
////    this->covMotionError->at(1, 1) = (*Moments::getInstance()->getMotion()).variances[Y]->getFilteredVariance();
////    this->covMotionError->at(2, 2) = (*Moments::getInstance()->getMotion()).variances[THETA]->getFilteredVariance();
//
//    // I think this may be wrong
//    cov.at(0, 1) = (*Moments::getInstance()->getMotion()).covariances[XY]->getFilteredCovariance();
//    cov.at(1, 0) = (*Moments::getInstance()->getMotion()).covariances[XY]->getFilteredCovariance();
//    cov.at(0, 2) = (*Moments::getInstance()->getMotion()).covariances[XZ]->getFilteredCovariance();
//    cov.at(2, 0) = (*Moments::getInstance()->getMotion()).covariances[XZ]->getFilteredCovariance();
//    cov.at(1, 2) = (*Moments::getInstance()->getMotion()).covariances[YZ]->getFilteredCovariance();
//    cov.at(2, 1) = (*Moments::getInstance()->getMotion()).covariances[YZ]->getFilteredCovariance();
//    return cov;
//}

void Seif::resolveActiveFeats() {
    unsigned long startIdx;
    this->F_I->zeroMatrix();
    for (long i = 0; i < std::fmin(this->featuresFound, this->maxActiveFeatures); i++) {
        FEATURE *feature = &((*this->activeFeatures)[i]);
        if (!feature) {
            break;
        }
        startIdx = featIdx(feature->idx);
        this->F_I->at(X, startIdx) = 1;
        this->F_I->at(Y, startIdx + 1) = 1;

        Matrix<float> quadMat = MatrixManipulator::getInstance()->quadratic(&(*this->F_I), &(*this->informationMatrix));
        quadMat.invert();
        Matrix<float> outer = MatrixManipulator::getInstance()->multiply(&quadMat, &(*this->F_I));

        Matrix<float> infoVec = *this->informationVector;
        Matrix<float> prodMat = MatrixManipulator::getInstance()->multiply(&(*this->informationMatrix), &(*this->stateEstimate));
        MatrixManipulator::getInstance()->subtract(&infoVec, &prodMat);

        Matrix<float> fi_T = *this->F_I; fi_T.transpose();
        Matrix<float> intrMat = MatrixManipulator::getInstance()->multiply(&(*this->informationMatrix), &fi_T);
        intrMat = MatrixManipulator::getInstance()->multiply(&intrMat, &(*this->F_I));
        intrMat = MatrixManipulator::getInstance()->multiply(&intrMat, &(*this->stateEstimate));

        MatrixManipulator::getInstance()->add(&infoVec, &intrMat);
        Matrix<float> stEst = MatrixManipulator::getInstance()->multiply(&outer, &infoVec);
        this->stateEstimate->at(startIdx) = stEst.at(X);
        this->stateEstimate->at(startIdx + 1) = stEst.at(Y);

        if (this->printMatrices) {
            std::cout << "State Estimate (Active):" << std::endl;
            this->stateEstimate->print();
        }
    }
}

void Seif::resolveAllFeats(const Matrix<float> *stateEstimate) {
    Matrix<float> quadMat = MatrixManipulator::getInstance()->quadratic(&(*this->F_X), &(*this->informationMatrix));
    quadMat.invert();
    Matrix<float> outer = MatrixManipulator::getInstance()->multiply(&quadMat, &(*this->F_X));

    Matrix<float> infoVec = *this->informationVector;
    Matrix<float> prodMat = MatrixManipulator::getInstance()->multiply(&(*this->informationMatrix), stateEstimate);
    MatrixManipulator::getInstance()->subtract(&infoVec, &prodMat);

    Matrix<float> fx_T = *this->F_X; fx_T.transpose();
    Matrix<float> intrMat = MatrixManipulator::getInstance()->multiply(&(*this->informationMatrix), &fx_T);
    intrMat = MatrixManipulator::getInstance()->multiply(&intrMat, &(*this->F_X));
    intrMat = MatrixManipulator::getInstance()->multiply(&intrMat, stateEstimate);

    MatrixManipulator::getInstance()->add(&infoVec, &intrMat);
    Matrix<float> stEst = MatrixManipulator::getInstance()->multiply(&outer, &infoVec);
    this->stateEstimate->at(X) = stEst.at(X);
    this->stateEstimate->at(Y) = stEst.at(Y);
    this->stateEstimate->at(THETA) = stEst.at(THETA);

    if (this->printMatrices) {
        std::cout << "State Estimate (All):" << std::endl;
        this->stateEstimate->print();
    }
}

/**
 * @fn updateQ
 * @brief measurement covariance error
 * NOTE: not sure if we need to place RAY
 * @param incidentRay
 * @param correspondence
 */
//void Seif::updateQ(const RAY &incidentRay, const float &correspondence) {
//    this->Q->zeroMatrix();
//    this->Q->at(0, 0) = (*Moments::getInstance()->getMeasurement()).variances[RANGE]->getFilteredVariance();
//    this->Q->at(1, 1) = (*Moments::getInstance()->getMeasurement()).variances[ANGLE]->getFilteredVariance();
//    this->Q->at(2, 2) = (*Moments::getInstance()->getMeasurement()).variances[CORRESPONDENCE]->getFilteredVariance();
//
//    // Pretty sure this is wrong
////    this->Q->at(0, 1) = (*Moments::getInstance()->getMeasurement()).covariances[XY]->getFilteredCovariance();
////    this->Q->at(1, 0) = (*Moments::getInstance()->getMeasurement()).covariances[XY]->getFilteredCovariance();
////    this->Q->at(0, 2) = (*Moments::getInstance()->getMeasurement()).covariances[XZ]->getFilteredCovariance();
////    this->Q->at(2, 0) = (*Moments::getInstance()->getMeasurement()).covariances[XZ]->getFilteredCovariance();
////    this->Q->at(1, 2) = (*Moments::getInstance()->getMeasurement()).covariances[YZ]->getFilteredCovariance();
////    this->Q->at(2, 1) = (*Moments::getInstance()->getMeasurement()).covariances[YZ]->getFilteredCovariance();
//}

void Seif::updateDeltaPos(const POSE &pose) {
    this->deltaPosition->at(X) = pose.x - rPose.x;
    this->deltaPosition->at(Y) = pose.y - rPose.y;
}

void Seif::update_q() {
    Matrix<float> dp_T = *this->deltaPosition; dp_T.transpose();
    Matrix<float> resMat = MatrixManipulator::getInstance()->multiply(&dp_T, &(*this->deltaPosition));
    this->q = resMat.at(0);
}

void Seif::updateZHat(const float &correspondence) {
    this->zHat->at(RANGE) = sqrt(this->q);
    this->zHat->at(ANGLE) = atan2(this->deltaPosition->at(Y), this->deltaPosition->at(X)) - rPose.theta;
    this->zHat->at(CORRESPONDENCE) = correspondence;
}

void Seif::updateH(const unsigned long &idx) {
    this->H->at(X, X) = (sqrt(this->q) * this->deltaPosition->at(X)) / this->q;
    this->H->at(X, Y) = ((-1) * sqrt(this->q) * this->deltaPosition->at(Y)) / this->q;
    this->H->at(Y, X) = (this->deltaPosition->at(Y)) / this->q;
    this->H->at(Y, Y) = (this->deltaPosition->at(X)) / this->q;
    this->H->at(Y, 2) = (-1) / this->q;

    unsigned long startIdx = featIdx(idx);

    this->H->at(X, startIdx + X) = ((-1) * sqrt(this->q) * this->deltaPosition->at(X)) / this->q;
    this->H->at(X, startIdx + Y) = (sqrt(this->q) * this->deltaPosition->at(Y)) / this->q;
    this->H->at(Y, startIdx + X) = ((-1) * this->deltaPosition->at(Y)) / this->q;
    this->H->at(Y, startIdx + Y) = ((-1) * this->deltaPosition->at(X)) / this->q;
    this->H->at(2, startIdx + 2) = (1) / this->q;
}

void Seif::updateEpsilon(const FEATURE &feature) {
    Matrix<float> stateEstimate = *this->stateEstimate;
    Matrix<float> prodMat = MatrixManipulator::getInstance()->multiply(&(*this->H), &stateEstimate);

    Matrix<float> z_i{feature.incidentRay.range, feature.incidentRay.angle, feature.correspondence};
    MatrixManipulator::getInstance()->subtract(&z_i, &(*this->zHat));
    MatrixManipulator::getInstance()->subtract(&z_i, &prodMat);

    Matrix<float> Q_inv = *this->measurementCov; Q_inv.invert();
    Matrix<float> H_T = *this->H; H_T.transpose();
    z_i = MatrixManipulator::getInstance()->multiply(&Q_inv, &z_i);
    stateEstimate = MatrixManipulator::getInstance()->multiply(&H_T, &z_i);
    MatrixManipulator::getInstance()->add(&(*this->informationVector), &stateEstimate);
}

void Seif::updateOmega() {
    Matrix<float> Q_inv = *this->measurementCov; Q_inv.invert();
    Matrix<float> infoMat = MatrixManipulator::getInstance()->quadratic(&(*this->H), &Q_inv); /** Q is already in inverse form. */
    MatrixManipulator::getInstance()->add(&(*this->informationMatrix), &infoMat);
}

void Seif::remodel(FEATURE &feature, const RAY &incidentRay) {
    std::array<float, 2> xyCoords = Equations::getInstance()->originToPoint(
            incidentRay,
            {(*this->stateEstimate).at(X), (*this->stateEstimate).at(Y), (*this->stateEstimate).at(THETA)},
            true);

    feature.incidentRay = incidentRay;
    feature.pose = {.x = xyCoords[X], .y = xyCoords[Y]};
    feature.correspondence = Equations::getInstance()->cantor(xyCoords[X], xyCoords[Y]);
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
    u_long back = this->featuresFound;

    while(front <= back) {
        unsigned long position = (front && back) ? front + (back - 1) / 2 : 0;
        relation section = comparison(correspondence, (*this->recordedFeatures)[position].correspondence);
        switch (section) {
            case EQUIV:
                return true;
            case LOWER:
                back = --position;
                break;
            case HIGHER:
                front = ++position;
                break;
            default:
                std::cout << "Error: bad feature identifier comparison <" << section << ">" << std::endl;
                exit(EXIT_FAILURE);
        }
    }
    return false;
}

void Seif::addFeature(FEATURE &feature) {
    u_long idx = this->featuresFound;
    feature.idx = idx;
    if (this->isActiveFull()) {
        *this->toDeactivate = (*this->activeFeatures)[idx = 0]; // Keep the furthest away feature in the initial position;
    }
    (*this->activeFeatures)[idx] = feature;
    (*this->recordedFeatures)[this->featuresFound++] = feature;

    // TODO: maybe should mark feature on info mat and vec

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

void Seif::updateInformationMatrix() {
    Matrix<float> m0 = this->defineProjection(&(this->toDeactivate->idx), false);
    Matrix<float> xm0 = this->defineProjection(&(this->toDeactivate->idx));
    this->makeInactive(&(*this->toDeactivate));

    // F_m_0
    Matrix<float> infoM0 = this->resolveProjection(&m0);
    // F_x,m_0
    Matrix<float> infoXM0 = this->resolveProjection(&xm0);
    // F_x
    Matrix<float> infoX = this->resolveProjection(&(*this->F_X));

    Matrix<float> infoMatrix = *this->informationMatrix;
    MatrixManipulator::getInstance()->subtract(&infoMatrix, &infoM0);
    MatrixManipulator::getInstance()->add(&infoMatrix, &infoXM0);
    MatrixManipulator::getInstance()->subtract(&infoMatrix, &infoX);
    *this->informationMatrix = infoMatrix;
}

void Seif::updateInformationVector(const Matrix<float> *prevInfoMat) {
    Matrix<float> infoMat = *this->informationMatrix;
    MatrixManipulator::getInstance()->subtract(&infoMat, prevInfoMat);
    Matrix<float> res = MatrixManipulator::getInstance()->multiply(&(*this->stateEstimate), &infoMat);
    MatrixManipulator::getInstance()->add(&(*this->informationVector), &res);
}

Matrix<float> Seif::resolveProjection(const Matrix<float> *projection) {
    Matrix<float> quadMat = MatrixManipulator::getInstance()->quadratic(projection, &(*this->informationMatrix));
    quadMat.invert();

    Matrix<float> intr = MatrixManipulator::getInstance()->quadratic(projection, &quadMat);
    Matrix<float> leftHS = MatrixManipulator::getInstance()->multiply(&(*this->informationMatrix), &intr);
    return MatrixManipulator::getInstance()->multiply(&leftHS, &(*this->informationMatrix));
}

Matrix<float> Seif::defineProjection(const unsigned long *idx, const bool &includePose) {
    Matrix<float> projection(this->F_X->numRows(), this->F_X->numCols());
    if (includePose) {
        projection.at(X, X) = 1;
        projection.at(Y, Y) = 1;
        projection.at(THETA, THETA) = 1;
    }
    if (idx) {
        unsigned long startIdx = featIdx(*idx);
        projection.at(X, startIdx) = 1;
        projection.at(Y, startIdx + Y) = 1;
        projection.at(2, startIdx + 2) = 1;
    }

    return projection;
}

void Seif::makeInactive(FEATURE *toDeact) {
    toDeact = nullptr;
}

Matrix<float> Seif::identity(const unsigned long &size) {
    Matrix<float> matrix(size, size);
    for (u_long i = 0; i < size; i++) {
        matrix.at(i, i) = 1;
    }
    return matrix;
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
