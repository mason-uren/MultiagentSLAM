//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_DETECTION_H
#define C_DETECTION_H

#include <array>
#include <IncidentRayInterface.h>
#include <SLAMConfigIn.h>
#include <FIRFilter.h>

typedef struct {
    float left[FILTER_LENGTH]{};
    float center[FILTER_LENGTH]{};
    float right[FILTER_LENGTH]{};
} SONAR_FILTER_BUFS;

class Detection : public IncidentRayInterface {
public:
    explicit Detection(DETECTION_CONFIG *detectionConfig) :
        uppderDetectLimit(detectionConfig->highDetectionBoundaryInM),
        sonarRangeInM(detectionConfig->sonarRangeInM),
        incidentAngles(new std::array<float, RANGE_SENSOR_COUNT>
            {
                (detectionConfig->sonarCoverageInRad / 2),
                (0),
                (-detectionConfig->sonarCoverageInRad / 2)
            }),
        incidentRay(new RAY {.range = detectionConfig->sonarRangeInM, .angle = 0}),
        buffers(new SONAR_FILTER_BUFS()),
        leftSonarFilter(new FIRFilter<float, FILTER_LENGTH>(buffers->left)),
        centerSonarFilter(new FIRFilter<float, FILTER_LENGTH>(buffers->center)),
        rightSonarFilter(new FIRFilter<float, FILTER_LENGTH>(buffers->right))
        {}

    ~Detection() override = default;

    bool hasIncidentRay() override;
    RAY *getIncidentRay() override;

    void MLIncidentRay(const std::array<SONAR, 3> &sonar);

private:
    void setIncidentRay(RAY ray) override;

    void addToFilter(const SONAR &ray);
    void inverseRayWeighting();

    /**
     * Variables
     */
    float uppderDetectLimit;
    float sonarRangeInM;
    std::shared_ptr<const std::array<float, RANGE_SENSOR_COUNT>> incidentAngles;
    std::shared_ptr<RAY> incidentRay;
    std::shared_ptr<SONAR_FILTER_BUFS> buffers;
    std::shared_ptr<FIRFilter<float, FILTER_LENGTH>> leftSonarFilter;
    std::shared_ptr<FIRFilter<float, FILTER_LENGTH>> centerSonarFilter;
    std::shared_ptr<FIRFilter<float, FILTER_LENGTH>> rightSonarFilter;
};


#endif //C_DETECTION_H
