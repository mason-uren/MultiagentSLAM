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
        maxActiveFeatures(seifConfig->maxActiveFeatures),
        minFeatureDist(seifConfig->featureDistInM),
        recordedFeatures(new std::vector<FEATURE>(seifConfig->maxFeatures, FEATURE{})),
        activeFeatures(new std::vector<FEATURE>((u_long) maxActiveFeatures, FEATURE{})),
        toDeactivate(new FEATURE{}),
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
//        Q(new Matrix<float>(ELEMENT_SIZE, ELEMENT_SIZE)),
        deltaPosition(new Matrix<float>(ELEMENT_SIZE - 1)),
        q(0),
        zHat(new Matrix<float>(ELEMENT_SIZE)),
        H(new Matrix<float>(ELEMENT_SIZE, N))
    {
        for (u_long i = 0; i < ELEMENT_SIZE; i++) {
            F_X->at(i, i) = 1;
            motionCov->at(i, i) = seifConfig->R;
            measurementCov->at(i, i) = seifConfig->Q;

            // Pretty sure I don't need
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

private:
    /*
     * Functions
     */
//    float *roverPose();

    // For testing purposes only
    bool printMatrices = true;

    void initializeMean(const VELOCITY &velocity);

    // Motion Update (func)
    void updateDeltaDel(const VELOCITY &velocity);
    void updatePsi();
    void updateLambda();
    void updatePhi();
    void updateKappa();
    void updateOmegaBar();
    void updateEpsilonBar();
    void updateMuBar();
//    Matrix<float> motionCovError();

    // State Estimate (func)
    void resolveActiveFeats();
    void resolveAllFeats(const Matrix<float> *stateEstimate);

    // Measurement Update (func)
//    void updateQ(const RAY &incidentRay, const float &correspondence);
    void updateDeltaPos(const POSE &pose);
    void update_q();
    void updateZHat(const float &correspondence);
    void updateH(const unsigned long &idx);
    void updateEpsilon(const FEATURE &feature);
    void updateOmega();
    void remodel(FEATURE &feature, const RAY &incidentRay);
    bool hasBeenObserved(const float &correspondence);
    void addFeature(FEATURE &feature);
    void organizeFeatures();
    relation comparison(const float &identifier, const float &otherID);
    bool isActiveFull();

    // Sparsification (func)
    void updateInformationMatrix();
    void updateInformationVector(const Matrix<float> *prevInfoMat);
    Matrix<float> resolveProjection(const Matrix<float> *projection);
    Matrix<float> defineProjection(const unsigned long *idx, const bool &includePose = true);
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
    const unsigned long N;
    unsigned long featuresFound;
    int maxActiveFeatures;
    float minFeatureDist;
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
//    std::shared_ptr<Matrix<float>> Q;
    std::shared_ptr<Matrix<float>> deltaPosition;
    float q;
    std::shared_ptr<Matrix<float>> zHat;
    std::shared_ptr<Matrix<float>> H;
};

#endif //C_SEIF_H
