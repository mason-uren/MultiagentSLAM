#include <memory>

#include <iostream>
#include <sstream>
#include <cmath>
#include <Eigen/Dense>


#include <MeanFilter.h>
#include <VarianceFilter.h>
#include <CovarianceFilter.h>
#include <Matrix.h>
#include <MatrixManipulator.h>

#include "Agent/Rover/RoverFactory.h"
#include "Slam/SlamAdapter/SlamAdapter.h"
#include "Utilities/SharedMemory/SharedMemory.h"
#include "Utilities/ConfigParser/ConfigParser.h"
#include "Utilities/Equations/Equations.h"



void loadDefaultConfig();
void jsonInitialize();

// Verification Functions
void printActiveRovers();
void testMeanFilter();
void testVarianceFilter();
void testFIRFilter();
void testTransformations();
void testMoments();
void testEquations();
void testRedBlackTree();
void testDetections();
void testMatrix();
void testMatrixMan();
void testSeif();

static std::shared_ptr<SharedMemory> sharedMemory;
static std::shared_ptr<ConfigParser> configParser;
static std::shared_ptr<Seif> seif;
static std::shared_ptr<Detection> detection;
static std::shared_ptr<RedBlackTree> localMap;
static std::shared_ptr<SYS_CONFIG_IN> systemConfig;


using json = nlohmann::json;

int main() {

    loadDefaultConfig();
    jsonInitialize(); // All initialization should occur here.

//    printActiveRovers();
//    testMeanFilter();
//    testVarianceFilter();
//    testFIRFilter();
//    testTransformations();
//    testMoments();
//    testEquations();
//    testRedBlackTree();
//    testDetections();
//    testMatrix();
//    testMatrixMan();
    testSeif();
    return 0;
}

void loadDefaultConfig() {
    json jsonFileConfig;
    systemConfig = std::make_shared<SYS_CONFIG_IN>();
    configParser = std::make_shared<ConfigParser>();
    sharedMemory = std::make_shared<SharedMemory>();

    // Working directory valid at 'src' (root)
    std::string filePath = "Config/slam_in.json";

    if (!configParser->loadJSONFromFile(filePath, &jsonFileConfig)) {
        std::stringstream root;
        root << filePath << " is missing. ";
        std::cerr << root.str() << strerror(errno);
        exit(1);
    }
    configParser->parseConfig(&(*systemConfig), &jsonFileConfig);
    systemConfig->block_id++;
    sharedMemory->writeMemoryIn(&(*systemConfig));
    std::cout << "Configuration Parsed." << std::endl;
}

void jsonInitialize() {
    while (true) {
        if (!sharedMemory->readMemoryIn(&(*systemConfig)) && systemConfig->config.hash != 0) {
            std::cout << "Configuration Loaded." << std::endl;
            break;
        }
        usleep(500);
    }

    SlamAdapter::getInstance();
    ActiveRovers::getInstance();
    Equations::getInstance();
    Moments::getInstance();
    MatrixManipulator::getInstance();


    // TODO : 'callbacks' will need to be setup before becoming active

    // Need to populate activeRovers and build translation based on number of rovers.
    SLAM_CONFIG slamConfig = systemConfig->config.slamConfig;
    if (slamConfig.valid) {
        for (unsigned int rover_id = 0; rover_id < slamConfig.numberOfRovers; rover_id++) {
            ROVER_CONFIG roverConfig = slamConfig.rovers[rover_id];
            if (roverConfig.valid) {
                roverConfig.ID = rover_id;
                Rover *rover = RoverFactory::create(&roverConfig);
                DETECTION_CONFIG detectionConfig = slamConfig.detectionConfig;
                SEIF_CONFIG seifConfig = slamConfig.seifConfig;
                LOCAL_MAP_CONFIG localMapConfig = slamConfig.localMapConfig;

                if (roverConfig.live &&
                    (detectionConfig.valid && seifConfig.valid && localMapConfig.valid)) {
                    detection = std::make_shared<Detection>(&detectionConfig);
                    seif = std::make_shared<Seif>(&seifConfig);
                    localMap = std::make_shared<RedBlackTree>(&localMapConfig);
                    rover->addDetection(&(*detection));
                    rover->addSeif(&(*seif));
                    rover->addLocalMap(&(*localMap));
                }

                ActiveRovers::getInstance()->addRover(*rover);
                SlamAdapter::getInstance()->addTransformation(roverConfig.name, new Transformation());
            }
        }
    }
}





/**
 * Testing Functions
 */
void printActiveRovers() {
    std::unordered_map<std::string, Rover *> rovers = *ActiveRovers::getInstance()->getActiveRovers();
    for (auto &rover : rovers) {
        std::cout << "Rover" << std::endl;
        std::cout << "Name : " << rover.second->getName() << std::endl;
        std::cout << "CORRESPONDENCE : " << rover.second->getID() << std::endl;
        std::cout << "Confidence : " << rover.second->getConfidence() << std::endl;
        std::cout << "Pose : x - " << rover.second->getCurrentPose()->x <<
            " y - " << rover.second->getCurrentPose()->y <<
            " theta - " << rover.second->getCurrentPose()->theta << std::endl;
        if (rover.second->getVelocity() != nullptr) {
            std::cout << "Vel : linear - " << rover.second->getVelocity()->linear <<
                      " angular - " << rover.second->getVelocity()->angular << std::endl;
        }
    }
}

void testMeanFilter() {
    float expected = 0;
    std::array<float, 5> vals_1 {1, 2, 3, 4, 5};
    MeanFilter<float> *meanFilter = new MeanFilter<float>();
    for (auto val : vals_1) {
        expected += val;
        float i = meanFilter->onlineAverage(val);
    }
    expected /= vals_1.size();
    std::cout << "Mean Filter (" <<
         ((expected == meanFilter->getFilteredValue()) ?
            "PASS" : "FAIL") <<
    ")" << std::endl;
}

void testVarianceFilter() {
    float expected = 0;
    float mean = 0;
    std::array<float, 5> vals {2, 6, 6, 3, 6};
    MeanFilter<float> *meanFilter = new MeanFilter<float>();
    VarianceFilter<float> *varianceFilter = new VarianceFilter<float>();
    for (auto val : vals) {
        mean += val;
    }
    mean /= vals.size();
    for (auto val : vals) {
        expected += std::pow(val - mean, 2);
        varianceFilter->onlineVariance(val, meanFilter->onlineAverage(val));
    }
    expected /= vals.size();
    std::cout << "Variance Filter (" <<
         ((abs(expected - varianceFilter->getFilteredVariance()) < 0.1) ?
          "PASS" : "FAIL") <<
    ")" << std::endl;
}

void testFIRFilter() {
    float mean = 0;
    std::array<float, 5> vals {3, 3, 3, 3, 10};
    float buf[FILTER_LENGTH];
    FIRFilter<float, FILTER_LENGTH> *fir = new FIRFilter<float, FILTER_LENGTH>(buf);
    for (auto val : vals) {
        mean += val;
        float i = fir->filterValue(val);
    }
    mean /= vals.size();
    std::cout << "FIRFilter (" <<
          ((mean - fir->getValue() < 3) ?
           "PASS" : "FAIL") <<
    ")" << std::endl;
}

void testTransformations() {
    std::unordered_map<std::string, Transformation *> *temp_map = SlamAdapter::getInstance()->getTransformations();
    std::array<std::string, 3> rovers_names {"achilles", "aeneas", "ajax"};
    for (auto &val : rovers_names) {
        (*temp_map)[val]->x_translation->filterValue(3.3);
        (*temp_map)[val]->y_translation->filterValue(3.3);
        (*temp_map)[val]->orientation->filterValue(3.3);
        std::cout << "Rover - " << val << std::endl;
        std::cout << " X - " << (*temp_map)[val]->x_translation->getValue() <<
        " Y - " << (*temp_map)[val]->y_translation->getValue() <<
        " ORIENT - " << (*temp_map)[val]->orientation->getValue() << std::endl;
    }
}

void testMoments() {
    std::array<MeanFilter<float> *, 3> means = (*Moments::getInstance()->getMotion()).means;
    std::array<VarianceFilter<float> *, 3> var = (*Moments::getInstance()->getMotion()).variances;
    std::array<CovarianceFilter<float> *, 3> covariances = (*Moments::getInstance()->getMotion()).covariances;;

    std::vector<long> x_vals {1, 2, 3};
    std::vector<long> y_vals {4, 5, 6};

    for (size_t i = 0; i < 3; i++) {
        means[X]->onlineAverage(x_vals[i]);
        means[Y]->onlineAverage(y_vals[i]);
        var[X]->onlineVariance(x_vals[i], means[X]->getFilteredValue());
        var[Y]->onlineVariance(y_vals[i], means[Y]->getFilteredValue());
        covariances[0]->onlineCovariance(x_vals[i], y_vals[i],
                means[X]->getFilteredValue(), means[Y]->getFilteredValue());
    }
    std::cout << "Mean Filter: (" <<
        ((means[X]->getFilteredValue() == 2 && means[Y]->getFilteredValue() == 5) ? "PASS" : "FAIL") <<
        ")" << std::endl;
    std::cout << "Variance Filter (" <<
        ((var[X]->getFilteredVariance() > 0.6 && var[X]->getFilteredVariance() < 0.7) ? "PASS" : "FAIL") <<
        ")" << std::endl;
    std::cout << "Covariance Filter (" <<
        ((covariances[0]->getFilteredCovariance() > 0.6 && covariances[0]->getFilteredCovariance() < 0.7) ? "PASS" : "FAIL") <<
        ")" << std::endl;
}

void testEquations() {
    // Origin to Point
    RAY ray {.range = 5, .angle = 0.6435};
    std::array<float, 3> pose{2, 1, 0};
    std::array<float, 2> pt = Equations::getInstance()->originToPoint(ray, pose);
    std::cout << "Origin to pt. (" << ((abs(pt[0] - 5) < 0.1 && abs(pt[1] - 5) < 5) ? "PASS" : "FAIL") << ")" << std::endl;

    // Wrapping theta
    double largeRad =  M_PI + M_PI_2;
    double smallRad = M_PI_2;
    float largeWrap = Equations::getInstance()->wrapTheta(largeRad);
    float smallWrap = Equations::getInstance()->wrapTheta(smallRad);
    std::cout << "Wrapping (" << ((fabs(largeWrap) - 1.58) < 0.1 && (M_PI_2 - fabs(smallWrap) < 0.1) ? "PASS" : "FAIL") << ")" << std::endl;

    // Normalize Value
    float norm = Equations::getInstance()->normalizeValue(3, 0, 10);
    std::cout << "Norm (" << ((norm > 0.29 && norm < 0.31) ? "PASS" : "FAIL") << ")" << std::endl;

    // Centroid
    std::array<std::array<float, 2>, 3> pairs = {};
    pairs.at(0) = {0, 0};
    pairs.at(1) = {3, 0};
    pairs.at(2) = {3, 4};
    std::array<float, 2> result = Equations::getInstance()->centroid(pairs);
    std::cout << "Centroid (" << ((result[0] == 2 && result[1] < 1.4 && result[1] > 1.2) ? "PASS" : "FAIL") << ")" <<std::endl;

    // Cantor
    std::cout << "Cantor (" <<
        ((Equations::getInstance()->cantor(1, -3) == Equations::getInstance()->cantor(1.00001, -3)) ?
        "FAIL" : "PASS") << ")" << std::endl;

    // Distance between two points
    POSE temp = {.x = -1, .y = -7, .theta = 0};
    POSE other = {.x = 2, .y = -3, .theta = 0};
    std::cout << "Distance between pts (" <<
        ((Equations::getInstance()->distBetweenPts(temp, other) == 5) ? "PASS" : "FAIL") <<
        ")" << std::endl;
}

void testRedBlackTree() {
    Rover rover = Rover();
    std::vector<double> sampleSignatures {
        1, 2.0, -3, 4, -5, -10.33, 11.5, -50.7, 7, -6
    };
    CLASSIFIER dummyClassifier {.area = 5, .orientation = 12.5, .signature = 11};
    std::vector<CLASSIFIER> sampleClassifiers {
        CLASSIFIER(),
        CLASSIFIER(),
        CLASSIFIER(),
        CLASSIFIER(),
        CLASSIFIER(),

        CLASSIFIER(),
        CLASSIFIER{.area = 5, .orientation = 12, .signature = 11.5},
        CLASSIFIER(),
        CLASSIFIER(),
        CLASSIFIER(),
    };
    std::array<FEATURE, 3> sampleFeatures {
        FEATURE{.pose.x = 0.3, .pose.y = 1.57, .incidentRay.angle = 3.14},
        FEATURE(),
        FEATURE()
    };

    for (int i = 0; i < sampleClassifiers.size(); i++) {
        sampleClassifiers[i].signature = (float) sampleSignatures[i];
    }

    // Create tree
    if (ActiveRovers::getInstance()->getRoverByName("achilles", rover)) {
        for (auto classifier : sampleClassifiers) {
            (*rover.getLocalMap()).addToTree(classifier, sampleFeatures);
        }
    }

    // Check if tree is balanced
    std::cout << "Root : " << *(*rover.getLocalMap()).getRoot() << std::endl;
    (*rover.getLocalMap()).printTree(new NODE_PTR{.valid = true, .node_ptr = *(*rover.getLocalMap()).getRoot()}, 0);

    // Check ML classifier (looking for like features)
    unsigned long index = 0;
    (*rover.getLocalMap()).findMLClassifier(dummyClassifier, index);
    std::cout << "Find Classifier (" << ((index == 6) ? "PASS" : "FAIL") << ")" << std::endl;

    // Get features from node
    std::array<FEATURE, 3> featuresToPop {
        FEATURE(),
        FEATURE(),
        FEATURE(),
    };

    // Testing whether features are passed by reference
    rover.getLocalMap()->getFeaturesFromNode(featuresToPop, index);
    float testVal = featuresToPop[0].pose.x;
    featuresToPop[0].pose.x = -1.2;
    std::cout << "Get features from node (" <<
        ((testVal == sampleFeatures[0].pose.x && sampleFeatures[0].pose.x != -1.2) ?
        "PASS" : "FAIL") << ")" << std::endl;

    // Reset Tree
    (*rover.getLocalMap()).resetTree();
    std::cout << "Reset Tree ("  << ((*rover.getLocalMap()->getRoot()) ? "FAIL" : "PASS") << ")" << std::endl;
}

void testDetections() {
    std::array<SONAR, 3> sampleSonar {
        SONAR{.id = sonar_id::LEFT, .observedRange = -1},
        SONAR{.id = sonar_id::CENTER, .observedRange = 1},
        SONAR{.id = sonar_id ::RIGHT, .observedRange = 1}
    };

    Rover rover = Rover();
    if (ActiveRovers::getInstance()->getRoverByName("achilles", rover)) {
        rover.getDetections()->MLIncidentRay(sampleSonar);
        std::cout << "Detection : " << rover.getDetections()->hasIncidentRay() << std::endl;
        RAY *ray = rover.getDetections()->getIncidentRay();
    }
}

void testMatrix() {
    Matrix<float> matrix(2, 3);
    matrix.at(0, 0) = 1;
    matrix.at(0, 1) = 2;
    matrix.at(0, 2) = 3;
    matrix.at(1, 0) = 4;
    matrix.at(1, 1) = 5;
    matrix.at(1, 2) = 6;
    std::cout << "Matrix Template (" <<
        ((matrix.at(0, 0) == 1) ? "PASS" : "FAIL") << ")" << std::endl;
    matrix.print();

    matrix.transpose();
    std::cout << "Matrix Transpose (" <<
        ((matrix.at(0, 1) == 4 && matrix.at(2, 0) == 3) ? "PASS" : "FAIL") << ")" << std::endl;
    matrix.print();


    std::cout << "Zero Matrix" <<std::endl;
    matrix.zeroMatrix();
    matrix.print();

    // Testing inverting matrices (also determinant)
    std::cout << "Original Mat: " << std::endl;
    Matrix<float> toInv(3, 3);
    toInv.at(0, 0) = 3, toInv.at(0, 1) = 0, toInv.at(0, 2) = 2;
    toInv.at(1, 0) = 2, toInv.at(1, 1) = 0, toInv.at(1, 2) = -2;
    toInv.at(2, 0) = 0, toInv.at(2, 1) = 1, toInv.at(2, 2) = 1;

    toInv.invert();

    Matrix<float> inverse(3, 3);
    inverse.at(0, 0) = 0.2, inverse.at(0, 1) = 0.2, inverse.at(0, 2) = 0;
    inverse.at(1, 0) = -0.2, inverse.at(1, 1) = 0.3, inverse.at(1, 2) = 1;
    inverse.at(2, 0) = 0.2, inverse.at(2, 1) = -0.3, inverse.at(2, 2) = 0;

    toInv.print();

    std::cout << "Inverse (" <<
              ((toInv == inverse) ? "PASS" : "FAIL") << ")" << std::endl;

}

void testMatrixMan() {
    Matrix<float> matrix(3, 3);
    matrix.at(0, 0) = 0.2, matrix.at(0, 1) = 0.2, matrix.at(0, 2) = 0;
    matrix.at(1, 0) = -0.2, matrix.at(1, 1) = 0.3, matrix.at(1, 2) = 1;
    matrix.at(2, 0) = 0.2, matrix.at(2, 1) = -0.3, matrix.at(2, 2) = 0;
    // Testing adding matrices
    MatrixManipulator::getInstance()->add<float>(&matrix, &matrix);

    Matrix<float> addition(3, 3);
    addition.at(0, 0) = 0.4, addition.at(0, 1) = 0.4, addition.at(0, 2) = 0;
    addition.at(1, 0) = -0.4, addition.at(1, 1) = 0.6, addition.at(1, 2) = 2;
    addition.at(2, 0) = 0.4, addition.at(2, 1) = -0.6, addition.at(2, 2) = 0;

    std::cout << "Matrix Addition (" <<
         ((matrix == addition) ? "PASS" : "FAIL") << ")" << std::endl;

    // Testing subtracting matrices
    Matrix<float> empty(3, 3);
    MatrixManipulator::getInstance()->subtract<float>(&matrix, &addition);

    std::cout << "Matrix Subtraction (" <<
        ((matrix == empty) ? "PASS" : "FAIL") << ")" << std::endl;

    // Testing multiplying matrices
    Matrix<float> aMatrix(2, 3);
    aMatrix.at(0, 0) = 1, aMatrix.at(0, 1) = 2, aMatrix.at(0, 2) = 3;
    aMatrix.at(1, 0) = 4, aMatrix.at(1, 1) = 5, aMatrix.at(1, 2) = 6;

    Matrix<float> bMatrix(3, 2);
    bMatrix.at(0, 0) = 7, bMatrix.at(0, 1) = 8;
    bMatrix.at(1, 0) = 9, bMatrix.at(1, 1) = 10;
    bMatrix.at(2, 0) = 11, bMatrix.at(2, 1) = 12;

    Matrix<float> testMat(2, 2);
    testMat.at(0, 0) = 58, testMat.at(0, 1) = 64;
    testMat.at(1, 0) = 139, testMat.at(1, 1) = 154;

    Matrix<float> resMult = MatrixManipulator::getInstance()->multiply<float>(&aMatrix, &bMatrix);

    Matrix<float> mat_1(2, 2);
    mat_1.at(0, 0) = 1, mat_1.at(0, 1) = 2;
    mat_1.at(1, 0) = 3, mat_1.at(1, 1) = 4;

    Matrix<float> mat_2(2, 2);
    mat_1.at(0, 0) = 1, mat_1.at(0, 1) = 2;
    mat_1.at(1, 0) = 3, mat_1.at(1, 1) = 4;

    Matrix<float> mat_3(2, 2);
    mat_1.at(0, 0) = 7, mat_1.at(0, 1) = 10;
    mat_1.at(1, 0) = 15, mat_1.at(1, 1) = 22;

    Matrix<float> res = MatrixManipulator::getInstance()->multiply<float>(&mat_1, &mat_2);

    std::cout << "Matrix Multiplication (" <<
        ((resMult == testMat && res == mat_3) ? "PASS" : "FAIL") << ")" << std::endl;

    Matrix<float> quad = MatrixManipulator::getInstance()->quadratic<float>(&mat_1, &mat_2);
    mat_1.invert();

    Matrix<float> resQuad_1 = MatrixManipulator::getInstance()->multiply<float>(&res, &mat_1);

    std::cout << "Matrix Quadratic (" <<
    ((quad == resQuad_1) ? "PASS": "FAIL") <<
    ")" << std::endl;
}

void testSeif() {
    /*
     * Goal: drive the rover in a figure 8
     * Starting Pt: '^'
     * OBJ: '#'
     *      ___
     *  #  |___|
     *     |___|
     *     ^
     * Verify pose at each vertex.
     */
    POSE rPose;
    std::vector<RAY> rays {
            {.range = 1.3, .angle = (float) 0.57},
            {.range = 1.3, .angle = (float) 0.0}
    };

    std::vector<VELOCITY> control {
            {.linear = 0.3, .angular = 0}, // Drive Up
            {.linear = 0, .angular = -1.57}, // Turn Right
            {.linear = 0.3, .angular = 0}, // Drive Right
            {.linear = 0, .angular = 1.57}, // Turn Up
            {.linear = 0.3, .angular = 0}, // Drive Up

            {.linear = 0, .angular = 1.57}, // Turn Left
            {.linear = 0.3, .angular = 0}, // Drive Left
            {.linear = 0, .angular = 1.57}, // Turn Down
            {.linear = 0.3, .angular = 0}, // Drive Down
            {.linear = 0, .angular = 1.57}, // Turn Left

            {.linear = 0.3, .angular = 0}, // Drive Right
            {.linear = 0, .angular = -1.57}, // Turn Right
            {.linear = 0.3, .angular = 0}, // Drive Down
            {.linear = 0, .angular = -1.57}, // Turn Right
            {.linear = 0.3, .angular = 0}, // Drive Left

            {.linear = 0, .angular = -1.57}, // Turn Right
    };
    std::cout << "Initial Pose: 0, 0, 0" << std::endl;
    int j = 0;
    for (VELOCITY velocity : control) {

        for (int i = 0; i < 10; i++) {
            seif->motionUpdate(velocity);
            seif->stateEstimateUpdate();

            // Test Measurements
            if (j == 0) { // || j == 7) {
                if (rays[j == 7 ? 1 : 0].range < 2.5) {
                    seif->measurementUpdate(rays[j]);
                }
                float x = rays[j == 7 ? 1 : 0].range * sin(rays[j == 7 ? 1 : 0].angle);
                float y = rays[j == 7 ? 1 : 0].range * cos(rays[j == 7 ? 1 : 0].angle);
                float yp = y - velocity.linear;
                float phi = atan2(x, yp);
                float Rp = (float) sqrt(pow(x, 2) + pow(yp, 2));

                rays[j == 7 ? 1 : 0].range = Rp;
                rays[j == 7 ? 1 : 0].angle = phi;
            }
            seif->sparsification();
        }
        j++;

        static bool isDriving = false;
        std::cout << ((isDriving = !isDriving) ? "DRIVING -> " : "TURNING -> ");
        rPose = seif->getRoverPose();
        std::cout << rPose.x << ", " << rPose.y << ", " << rPose.theta << std::endl;
    }
    rPose = seif->getRoverPose();
    std::cout << "Final Pose: " << rPose.x << ", " << rPose.y << ", " << rPose.theta << std::endl;

    std::cout << "SEIF (" << ((fabs(rPose.x) < 0.01 && fabs(rPose.y) < 0.01 && fabs(rPose.theta) <0.01) ? "PASS" : "FAIL") << ")" << std::endl;
}

