#include <iostream>
#include <sstream>

#include "Utilities/SharedMemory/SharedMemory.h"
#include "Utilities/ConfigParser/ConfigParser.h"
#include "Agent/Rover/RoverFactory.h"
#include "Slam/SlamAdapter/SlamAdapter.h"
#include "Utilities/Filters/MeanFilter.h"
#include "Utilities/Filters/VarianceFilter.h"
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

std::shared_ptr<SharedMemory> sharedMemory;
std::shared_ptr<ConfigParser> configParser;
SYS_CONFIG_IN *systemConfig;

using json = nlohmann::json;

int main() {

    /**
     * TODO This initialization should not be occuring here.
     */
    sharedMemory = std::shared_ptr<SharedMemory>(new SharedMemory());
    configParser = std::shared_ptr<ConfigParser>(new ConfigParser());
    systemConfig = new SYS_CONFIG_IN();
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

    return 0;
}

void loadDefaultConfig() {
    json jsonFileConfig;

    // Working directory valid at 'src' (root)
    std::string filePath = "Config/slam_in.json";

    if (!configParser->loadJSONFromFile(filePath, &jsonFileConfig)) {
        std::stringstream root;
        root << filePath << " is missing. ";
        std::cerr << root.str() << strerror(errno);
        exit(1);
    }
    configParser->parseConfig(systemConfig, &jsonFileConfig);
    systemConfig->block_id++;
    sharedMemory->writeMemoryIn(systemConfig);
    std::cout << "Configuration Parsed." << std::endl;
}

void jsonInitialize() {
    while (true) {
        if (!sharedMemory->readMemoryIn(systemConfig) && systemConfig->config.hash != 0) {
            std::cout << "Configuration Loaded." << std::endl;
            break;
        }
        usleep(500);
    }

    SlamAdapter::getInstance();
    ActiveRovers::getInstance();
    Equations::getInstance();
    Moments::getInstance();


    // TODO : 'callbacks' will need to be setup before becoming active

    // Need to populate activeRovers and build translation based on number of rovers.
    std::shared_ptr<SLAM_CONFIG> slamConfig(&systemConfig->config.slamConfig);
    if (slamConfig->valid) {
        for (unsigned int rover_id = 0; rover_id < slamConfig->numberOfRovers; rover_id++) {
            ROVER_CONFIG rover = slamConfig->rovers[rover_id];
            if (rover.valid) {
                rover.ID = rover_id;

                /*
                 * if NOT live - create
                 * if live - must have valid detectionConfig and seifConfig
                 */
                if (!rover.live || (rover.detectionConfig.valid && rover.seifConfig.valid)) {
                    ActiveRovers::getInstance()->addRover(*RoverFactory::create(&rover));
                    SlamAdapter::getInstance()->addTransformation(rover.name, new Transformation());
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
        ((featuresToPop[0].incidentRay == sampleFeatures[0].incidentRay && sampleFeatures[0].xRelative != -1.2) ?
        "PASS" : "FAIL") << ")" << std::endl;

    // Reset Tree
    (*rover.getLocalMap()).resetTree();
    std::cout << "Reset Tree ("  << ((*rover.getLocalMap()->getRoot()) ? "FAIL" : "PASS") << ")" << std::endl;
}

