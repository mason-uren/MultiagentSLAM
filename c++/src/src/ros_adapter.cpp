#include <memory>

#include <iostream>
#include <sstream>

#include <MeanFilter.h>
#include <VarianceFilter.h>
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
//    testFIRFilter();f
//    testTransformations();
//    testMoments();
//    testEquations();
//    testRedBlackTree();
//    testDetections();
//    testMatrix();
    testMatrixMan();

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
                ActiveRovers::getInstance()->addRover(*RoverFactory::create(&roverConfig));
                SlamAdapter::getInstance()->addTransformation(roverConfig.name, new Transformation());

                DETECTION_CONFIG detectionConfig = slamConfig.detectionConfig;
                SEIF_CONFIG seifConfig = slamConfig.seifConfig;
                LOCAL_MAP_CONFIG localMapConfig = slamConfig.localMapConfig;
                if (roverConfig.live &&
                    (detectionConfig.valid && seifConfig.valid && localMapConfig.valid)) {
                    detection = std::make_shared<Detection>(&detectionConfig);
                    seif = std::make_shared<Seif>(&seifConfig);
                    localMap = std::make_shared<RedBlackTree>(&localMapConfig);

                    Rover *rover = new Rover();
                    ActiveRovers::getInstance()->getRoverByName(roverConfig.name, *rover);
                    rover->addDetection(&(*detection));
                    rover->addSeif(&(*seif));
                    rover->addLocalMap(&(*localMap));
                }
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
        std::cout << "ID : " << rover.second->getID() << std::endl;
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
         ((expected == varianceFilter->getFilteredVariance()) ?
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
    std::array<VarianceFilter<float> *, 3> *var = Moments::getInstance()->getVariances();
    std::array<MeanFilter<float> *, 3> *means = Moments::getInstance()->getMeans();
    std::array<std::string, 3> rovers_names {"achilles", "aeneas", "ajax"};

}

void testEquations() {
    // Origin to Point
    std::array<float, 2> ray {5, 1.0472};
    std::array<float, 3> pose{2, 1, 0};
    std::array<float, 2> pt = Equations::getInstance()->originToPoint(ray, pose);
    std::cout << "Origin to pt. (" << (((pt[0] > 6.3 && pt[0] < 6.4) && (pt[1] > 3.48 && pt[1] < 3.5)) ? "PASS" : "FAIL") << ")" << std::endl;

    // Wrapping theta
    double rad =  M_PI + M_PI_2;
    float val = Equations::getInstance()->wrapTheta(rad);
    std::cout << "Wrapping (" << (val > -1.58 && val < -1.56 ? "PASS" : "FAIL") << ")" << std::endl;

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
        ((Equations::getInstance()->cantor(1, -3) == Equations::getInstance()->cantor(-1, -3)) ?
        "FAIL" : "PASS") << ")" << std::endl;
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
        FEATURE{.xRelative = 0.3, .yRelative = 1.57, .incidentRay = 3.14},
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
    rover.getLocalMap()->getFeaturesFromNode(featuresToPop, index);

    featuresToPop[0].xRelative = -1.2;
    std::cout << "Get features from node (" <<
        ((featuresToPop[0].xRelative == sampleFeatures[0].xRelative && sampleFeatures[0].xRelative != -1.2) ?
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

    matrix.transpose();

    matrix.print();


    std::cout << "Matrix Transpose (" <<
        ((matrix.at(0, 1) == 4 && matrix.at(2, 0) == 3) ? "PASS" : "FAIL") << ")" << std::endl;

    std::cout << "Reset Matrix" <<std::endl;
    matrix.resetMatrix();
    matrix.print();

}

void testMatrixMan() {
    // Testing inverting matrices (also determinant)
    Matrix<float> matrix(3, 3);
    matrix.at(0, 0) = 3, matrix.at(0, 1) = 0, matrix.at(0, 2) = 2;
    matrix.at(1, 0) = 2, matrix.at(1, 1) = 0, matrix.at(1, 2) = -2;
    matrix.at(2, 0) = 0, matrix.at(2, 1) = 1, matrix.at(2, 2) = 1;

    MatrixManipulator::getInstance()->invert<float>(&matrix);

    Matrix<float> inverse(3, 3);
    inverse.at(0, 0) = 0.2, inverse.at(0, 1) = 0.2, inverse.at(0, 2) = 0;
    inverse.at(1, 0) = -0.2, inverse.at(1, 1) = 0.3, inverse.at(1, 2) = 1;
    inverse.at(2, 0) = 0.2, inverse.at(2, 1) = -0.3, inverse.at(2, 2) = 0;

    std::cout << "Inverse/Determinant/Adjoint/Cofactor (" <<
          ((matrix == inverse) ? "PASS" : "FAIL") << ")" << std::endl;

    // Testing adding matrices
    MatrixManipulator::getInstance()->add<float>(&matrix, &inverse);

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
    Matrix<float> resMult(2, 2);
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

    MatrixManipulator::getInstance()->multiply(&resMult, &aMatrix, &bMatrix);

    std::cout << "Matrix Multiplication (" <<
        ((resMult == testMat) ? "PASS" : "FAIL") << ")" << std::endl;

}

