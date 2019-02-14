//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SEIF_H
#define C_SEIF_H

#include <SharedMemoryStructs.h>
#include <SLAMConfigIn.h>
#include <array>
#include <vector>
#include <functional>
#include <iostream>

typedef std::function<void(FEATURE, float *)> FeatureCallback;

class Seif {
public:
    Seif(SEIF_CONFIG *seifConfig) :
            featuresFound(0),
            activeFeatureIndexes(new std::vector<long>(seifConfig->maxFeatures)),
            informationMatrix(new std::vector<std::vector<ELEMENT>>(seifConfig->maxFeatures,
                    std::vector<ELEMENT>(seifConfig->maxFeatures, ELEMENT()))),
            informationVector(new std::vector<ELEMENT>(seifConfig->maxFeatures)),
            stateEstimate(new std::vector<ELEMENT>(seifConfig->maxFeatures)),
            priorStateEstimate(new std::vector<ELEMENT>(seifConfig->maxFeatures)),
            F_X(new std::vector<std::array<ELEMENT, 3>>(seifConfig->maxFeatures)),
            delta(new std::array<float, 3>()),
            del(new std::array<std::array<float, 3>, 3>()),
            psi(new std::vector<std::vector<float>>(seifConfig->maxFeatures,
                    std::vector<float>(seifConfig->maxFeatures))),
            lambda(new std::vector<std::vector<float>>(seifConfig->maxFeatures,
                    std::vector<float>(seifConfig->maxFeatures))),
            phi(new std::vector<std::vector<float>>(seifConfig->maxFeatures,
                    std::vector<float>(seifConfig->maxFeatures))),
            kappa(new std::vector<std::vector<float>>(seifConfig->maxFeatures,
                    std::vector<float>(seifConfig->maxFeatures))),
            omega_bar(new std::vector<std::vector<ELEMENT>>(seifConfig->maxFeatures,
                    std::vector<ELEMENT>(seifConfig->maxFeatures, ELEMENT()))),
            epsilon_bar(new std::vector<ELEMENT>(seifConfig->maxFeatures)),
            mu_bar(new std::vector<ELEMENT>(seifConfig->maxFeatures)),
            F_I(new std::vector<std::array<int, 2>>(seifConfig->maxFeatures)),
            Q(new std::array<std::array<float, 3>, 3>()),
            deltaPosition(new std::array<float, 2>()),
            q(0.0),
            zHat(new std::array<float, 3>()),
            H(new std::vector<std::array<float, 3>>(seifConfig->maxFeatures))
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
    std::shared_ptr<std::vector<long>> activeFeatureIndexes;
    std::shared_ptr<std::vector<std::vector<ELEMENT>>> informationMatrix;
    std::shared_ptr<std::vector<ELEMENT>> informationVector;
    std::shared_ptr<std::vector<ELEMENT>> stateEstimate;
    std::shared_ptr<std::vector<ELEMENT>> priorStateEstimate;

    // Callback
    std::shared_ptr<FeatureCallback> callback;

    /*
     * Motion Update
     */
    std::shared_ptr<std::vector<std::array<ELEMENT, 3>>> F_X; // Transpose is being stored (different from text)
    std::shared_ptr<std::array<float, 3>> delta;
    std::shared_ptr<std::array<std::array<float, 3>, 3>> del;

    std::shared_ptr<std::vector<std::vector<float>>> psi;
    std::shared_ptr<std::vector<std::vector<float>>> lambda;
    std::shared_ptr<std::vector<std::vector<float>>> phi;
    std::shared_ptr<std::vector<std::vector<float>>> kappa;
    std::shared_ptr<std::vector<std::vector<ELEMENT>>> omega_bar;
    std::shared_ptr<std::vector<ELEMENT>> epsilon_bar;
    std::shared_ptr<std::vector<ELEMENT>> mu_bar;

    /*
     * State Estimate
     */
    std::shared_ptr<std::vector<std::array<int, 2>>> F_I; // Transpose is being stored (different from text)

    /*
     * Measurment
     */
    std::shared_ptr<std::array<std::array<float, 3>, 3>> Q;
    std::shared_ptr<std::array<float, 2>> deltaPosition;
    float q; // TODO : need to verify that initial value of 0.0 is acceptable
    std::shared_ptr<std::array<float, 3>> zHat;
    std::shared_ptr<std::vector<std::array<float, 3>>> H; // Transpose is being stored (different from text)

};


#endif //C_SEIF_H
