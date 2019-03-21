//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SEIF_H
#define C_SEIF_H

#include <array>
#include <vector>
#include <functional>
#include <iostream>

#include <SharedMemoryStructs.h>
#include <SLAMConfigIn.h>
#include <Matrix.h>

typedef std::function<void(FEATURE, float *)> FeatureCallback;

class Seif {
public:
    explicit Seif(SEIF_CONFIG *seifConfig) :
        featuresFound(0),
        activeFeatureIndexes(new Matrix<u_long>(seifConfig->maxFeatures)),
        informationMatrix(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures),
                                            No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        informationVector(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        stateEstimate(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        priorStateEstimate(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        F_X(new Matrix<float>(No_POSE_ELEMS, No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        delta(new Matrix<float>(3)),
        del(new Matrix<float>(3, 3)),
        psi(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures),
                              No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        lambda(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures),
                                 No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        phi(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures),
                              No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        kappa(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures),
                                No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        omega_bar(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures),
                                    No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        epsilon_bar(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        mu_bar(new Matrix<float>(No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        F_I(new Matrix<float>(2, No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures))),
        Q(new Matrix<float>(3, 3)),
        deltaPosition(new Matrix<float>(2)),
        q(0),
        zHat(new Matrix<float>(3)),
        H(new Matrix<float>(3, No_POSE_ELEMS + (No_RAY_ELEMS * seifConfig->maxFeatures)))
    {}
    ~Seif() = default;

    void connectFeatureCallback(FeatureCallback &callback);
    void motionUpdate(POSE &pose, VELOCITY &velocity);
    POSE *updateStateEstimate();

    // Run on separate Thread
    void measurementUpdate(RAY &incidentRay);
    void sparsification();

private:
    bool isNewFeature(const RAY &incidentRay);
    float *roverPose();

    // Run on separate Thread
    void logFeature(const FEATURE &feature, std::array<float, 3> roverPose);

    /**
     * Variables
     */
    long featuresFound;
    std::shared_ptr<Matrix<unsigned long>> activeFeatureIndexes;
    std::shared_ptr<Matrix<float>> informationMatrix;
    std::shared_ptr<Matrix<float>> informationVector;
    std::shared_ptr<Matrix<float>> stateEstimate;
    std::shared_ptr<Matrix<float>> priorStateEstimate;

    // Callback
    std::shared_ptr<FeatureCallback> callback;

    /*
     * Motion Update
     */
    std::shared_ptr<Matrix<float>> F_X; // Transpose is being stored (different from text)
    std::shared_ptr<Matrix<float>> delta;
    std::shared_ptr<Matrix<float>> del;

    std::shared_ptr<Matrix<float>> psi;
    std::shared_ptr<Matrix<float>> lambda;
    std::shared_ptr<Matrix<float>> phi;
    std::shared_ptr<Matrix<float>> kappa;
    std::shared_ptr<Matrix<float>> omega_bar;
    std::shared_ptr<Matrix<float>> epsilon_bar;
    std::shared_ptr<Matrix<float>> mu_bar;

    /*
     * State Estimate
     */
    std::shared_ptr<Matrix<float>> F_I; // Transpose is being stored (different from text)

    /*
     * Measurment
     */
    std::shared_ptr<Matrix<float>> Q;
    std::shared_ptr<Matrix<float>> deltaPosition;
    float q; // TODO : need to verify that initial value of 0.0 is acceptable
    std::shared_ptr<Matrix<float>> zHat;
    std::shared_ptr<Matrix<float>> H; // Transpose is being stored (different from text)

};


#endif //C_SEIF_H
