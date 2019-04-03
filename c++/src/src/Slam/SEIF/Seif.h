//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SEIF_H
#define C_SEIF_H

#include <array>
#include <vector>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

#include <MatrixManipulator.h>
#include <SharedMemoryStructs.h>
#include <SLAMConfigIn.h>
#include <Matrix.h>

#include "../../Agent/Moments/Moments.h"

enum relation {
    EQUIV = 0,
    LOWER,
    HIGHER,
};

typedef std::function<void(FEATURE, float *)> FeatureCallback;

inline unsigned long featIdx(const unsigned long &idx) { return 3 * idx + 3; }

class Seif {
public:
    explicit Seif(SEIF_CONFIG *seifConfig) :
        N(ELEMENT_SIZE + (ELEMENT_SIZE * seifConfig->maxFeatures)),
        featuresFound(0),
        maxFeatures(seifConfig->maxFeatures),
        maxActiveFeatures(seifConfig->maxActiveFeatures),
        minFeatureDist(seifConfig->featureDistInM),
        maxCorrespondence(Equations::getInstance()->cantor(
                (seifConfig->featureDistInM * 2) * maxFeatures,
                (seifConfig->featureDistInM * 2)) * maxFeatures), // Diameter of feature marker
        recordedFeatures(new std::vector<FEATURE>(seifConfig->maxFeatures, FEATURE{})),
        activeFeatures(new std::vector<FEATURE>((u_long) maxActiveFeatures, FEATURE{})),
        toDeactivate(new FEATURE{.correspondence = -MAXFLOAT}),
        informationMatrix(new Matrix<float>(N, N)),
        informationVector(new Matrix<float>(N)),
        stateEstimate(new Matrix<float>(N)),
        motionCov(new Matrix<float>(ELEMENT_SIZE, ELEMENT_SIZE)),
        measurementCov(new Matrix<float>(ELEMENT_SIZE, ELEMENT_SIZE)),
        F_X(new Matrix<float>(ELEMENT_SIZE, N)),
        delta(new Matrix<float>(ELEMENT_SIZE)),
        del(new Matrix<float>(ELEMENT_SIZE, ELEMENT_SIZE)),
        psi(new Matrix<float>(N, N)),
        lambda(new Matrix<float>(N, N)),
        phi(new Matrix<float>(N, N)),
        kappa(new Matrix<float>(N, N)),
        F_I(new Matrix<float>((ELEMENT_SIZE - 1), N)),
        deltaPosition(new Matrix<float>(ELEMENT_SIZE - 1)),
        q(0),
        zHat(new Matrix<float>(ELEMENT_SIZE)),
        H(new Matrix<float>(ELEMENT_SIZE, N))
    {
        for (u_long i = 0; i < ELEMENT_SIZE; i++) {
            F_X->at(i, i) = 1;
            motionCov->at(i, i) = seifConfig->R[i];
            measurementCov->at(i, i) = seifConfig->Q[i];
            // Need to mark <x, y, theta> as observed
            informationMatrix->at(i, i) = 1;
        }
    }
    ~Seif() = default;

    void connectFeatureCallback(FeatureCallback &callback);
    void motionUpdate(const VELOCITY &velocity);
    POSE stateEstimateUpdate();

    // Run on separate Thread
    void measurementUpdate(const RAY &incidentRay);
    void sparsification();

    // For testing purposes only.
    POSE getRoverPose();
    void printRoverPose();

private:
    /*
     * Functions
     */
    // For testing purposes only
    bool printMatrices = false;

    // Motion Update (func)
    void updateDeltaDel(const VELOCITY &velocity);
    void updatePsi();
    void updateLambda();
    void updatePhi();
    void updateKappa();
    void updateOmegaBar();
    void updateEpsilonBar();
    void updateMuBar();

    // State Estimate (func)
    void integrateActiveFeatures();
    void generateStateEstimate(const Matrix<float> *stateEstimate);

    // Measurement Update (func)
    bool isNewFeature(const RAY &incidentRay);
    void deriveFeature(FEATURE &feature, const RAY &incidentRay);
    bool hasBeenObserved(const float &correspondence);
    void addFeature(FEATURE &feature);
    u_long &nextFeatureIndex();
    void organizeFeatures();
    relation comparison(const float &identifier, const float &otherID);
    bool isActiveFull();
    void updateDeltaPos(const POSE &featPose);
    void update_q();
    void updateZHat(const float &correspondence);
    void updateH(const unsigned long &idx);
    void infoVecSummation(Matrix<float> *infoVec, const FEATURE &feature);
    void infoMatrixSummation(Matrix<float> *infoMatrix);

    // Sparsification (func)
    void updateInformationMatrix();
    void updateInformationVector(const Matrix<float> *prevInfoMat);
    Matrix<float> resolveProjection(const Matrix<float> *projection);
    Matrix<float> defineProjection(const FEATURE *feat, const bool &includePose = true);
    void makeInactive(FEATURE *toDeact);

    // Tools
    Matrix<float> identity(const unsigned long &size);
    static bool correspondenceSort(const FEATURE &feat, const FEATURE &other);
    static bool distanceSort(const FEATURE &featA, const FEATURE &featB);

    // Run on separate Thread
    void logFeature(const FEATURE &feature, std::array<float, 3> roverPose);



    /**
     * Variables
     */
    static POSE rPose;

    const unsigned long N;
    unsigned long featuresFound;
    unsigned long maxFeatures;
    int maxActiveFeatures;
    float minFeatureDist;
    float maxCorrespondence;
    std::shared_ptr<std::vector<FEATURE>> recordedFeatures;
    std::shared_ptr<std::vector<FEATURE>> activeFeatures;
    std::shared_ptr<FEATURE> toDeactivate;
    std::shared_ptr<Matrix<float>> informationMatrix;
    std::shared_ptr<Matrix<float>> informationVector;
    std::shared_ptr<Matrix<float>> stateEstimate;

    std::shared_ptr<Matrix<float>> motionCov;
    std::shared_ptr<Matrix<float>> measurementCov;

    // Callback
    std::shared_ptr<FeatureCallback> callback;

    // Motion Update (vars)
    std::shared_ptr<Matrix<float>> F_X;
    std::shared_ptr<Matrix<float>> delta;
    std::shared_ptr<Matrix<float>> del;
    std::shared_ptr<Matrix<float>> psi;
    std::shared_ptr<Matrix<float>> lambda;
    std::shared_ptr<Matrix<float>> phi;
    std::shared_ptr<Matrix<float>> kappa;

    // State Estimate (vars)
    std::shared_ptr<Matrix<float>> F_I;

    // Measurment (vars)
    std::shared_ptr<Matrix<float>> deltaPosition;
    float q;
    std::shared_ptr<Matrix<float>> zHat;
    std::shared_ptr<Matrix<float>> H;
};

#endif //C_SEIF_H
