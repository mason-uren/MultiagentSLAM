#include "Adapter.h"

using json = nlohmann::json;
using lim_float = std::numeric_limits<float>;

using std::unique_ptr;
using std::make_unique;
using std::string;
using std::get;

int main() {

    loadDefaultConfig();
    jsonInitialize(); // All initialization should occur here.

//    testEnv();

    printActiveRovers();
    testMeanFilter();
    testVarianceFilter();
    testFIRFilter();
    testTransformations();
    testMoments();
    testEquations();
    testRedBlackTree();
    testDetections();
    testMatrix();
    testMatrixMan();
    testSeif();
//    testFeatureSet();
    return 0;
}

void loadDefaultConfig() {
    json jsonFileConfig;
    systemConfig = make_unique<SYS_CONFIG_IN>();
    configParser = make_unique<ConfigParser>();
    sharedMemory = make_unique<SharedMemory>();

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
    // TODO : why don't we need this
//    while (true) {
//        if (!sharedMemory->readMemoryIn(&(*systemConfig)) && systemConfig->config.hash != 0) {
//            std::cout << "Configuration Loaded." << std::endl;
//            break;
//        }
//        usleep(500);
//    }

    SlamAdapter::getInstance();
    ActiveRovers::getInstance();
    Equations::getInstance();
    Moments::getInstance();
    FeatureSet::getInstance();

    // Need to populate activeRovers and build translation based on number of rovers.
    SLAM_CONFIG slamConfig = systemConfig->config.slamConfig;
    if (slamConfig.valid) {
        for (unsigned int rover_id = 0; rover_id < slamConfig.numberOfRovers; rover_id++) {
            ROVER_CONFIG roverConfig = slamConfig.rovers[rover_id];
            if (roverConfig.valid) {
                roverConfig.ID = rover_id;
                Rover *rover = dynamic_cast<Rover *>(RoverFactory::create(&roverConfig));
                DETECTION_CONFIG detectionConfig = slamConfig.detectionConfig;
                SEIF_CONFIG seifConfig = slamConfig.seifConfig;
                LOCAL_MAP_CONFIG localMapConfig = slamConfig.localMapConfig;

                if (roverConfig.live &&
                    (detectionConfig.valid && seifConfig.valid && localMapConfig.valid)) {
                    detection = make_unique<Detection>(&detectionConfig);
                    seif = make_unique<Seif>(&seifConfig);
                    localMap = make_unique<RedBlackTree>(&localMapConfig);
                    rover->addDetection(&(*detection));
                    rover->addSeif(&(*seif));
                    rover->addLocalMap(&(*localMap));

                    roverName = rover->getName();
                }

                ActiveRovers::getInstance()->addRover(*rover);
                SlamAdapter::getInstance()->addTransformation(roverConfig.name, new Transformation());
            }
        }
    }
}

void kinematicsHandler(const POSE &pose, const VELOCITY &vel) {
    SlamAdapter::getInstance()->updateKinematics(roverName, pose, vel);
}

void sonarHandler(const std::array<SONAR, 3> &sonar) {
    SlamAdapter::getInstance()->updateDetections(roverName, sonar);
}

void slamHandler() {
    SlamAdapter::getInstance()->slamUpdate(roverName);
    publishBelief(*ActiveRovers::getInstance()->getRoverByName(roverName).getCurrentPose(),
                  ActiveRovers::getInstance()->getRoverByName(roverName).getConfidence());

}


// TODO: auxilaryRover Handler
void aRH() { // `auxilaryBeliefs`
    auto pose(POSE{});
    float confi = 0;
    // IMPORTANT: not completely accurate (should get rovers from param `auxilaryBeliefs`)
    for (auto rover : *ActiveRovers::getInstance()->getActiveRovers()) {
        // ROS_msg has update value feature that should also be checked
        if (rover.second->getName() != roverName) {
            pose = *rover.second->getCurrentPose();
            confi = rover.second->getConfidence();
            SlamAdapter::getInstance()->recordAuxilaryRoversBelief(rover.second->getName(), pose, confi);
        }
    }
}

// TODO: featureSetHandler
void fsH() { // `auxilaryFeatureSets`
    auto features{std::array<FEATURE, FEATURE_LIMIT>()};
    auto classifier{CLASSIFIER{}};
    // IMPORTANT: not completely accurate (should get rovers from param `auxilaryBeliefs`)
    for (auto rover : *ActiveRovers::getInstance()->getActiveRovers()) {
        // ROS_msg has update value feature that should also be checked
        string publisher{"Dummy publisher"};
        if (rover.second->getName() != roverName) {
            SlamAdapter::getInstance()->logAuxilaryFeatureSet(rover.second->getName(), features, classifier, publisher);
        }
    }
}

// TODO: transformationHandler
void tH() {
    auto transformation{POSE{}};
    // IMPORTANT: not completely accurate (should get rovers from param `auxilaryBeliefs`)
    for (auto rover : *ActiveRovers::getInstance()->getActiveRovers()) {
        // ROS_msg has update value feature that should also be checked
        if (rover.second->getName() != roverName) {
            string pairedRover{"Paired Rover"};
            SlamAdapter::getInstance()->updateTransformationByRover(transformation, pairedRover);
        }
    }
}

// TODO ROS stuff
void publishFeatureSet(const std::array<FEATURE, FEATURE_LIMIT> &featureSet, const CLASSIFIER &classifer) {
    std::cout << "GLOBAL PUBLISHER FEATURE SET" << std::endl;
    if (FeatureSet::getInstance()->readyToPublish()) {
        auto FS = FeatureSet::getInstance()->publishSet();
        publishFeatureSet(get<0>(FS), get<1>(FS));
        ActiveRovers::getInstance()->getRoverByName(roverName).integrateLocalFS(get<0>(FS), get<1>(FS));
    }
}

void publishBelief(const POSE &pose, const float &confi) {
    std::cout << "GLOBAL PUBLISHED BELIEF" << std::endl;
}

// TODO ROS stuff
void publishTransformation(const POSE &trans, const string &rID) {
    std::cout << "GLOBAL PUBLISHER TRANSLATION" << std::endl;
    if (ActiveRovers::getInstance()->getRoverByName(roverName).readyToPublish()) {
        // Publish new transformation
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
        std::cout << "id : " << rover.second->getID() << std::endl;
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
    std::array<MeanFilter<float> *, 3> means = (*Moments::getInstance()->motion).means;
    std::array<VarianceFilter<float> *, 3> var = (*Moments::getInstance()->motion).variances;
    std::array<CovarianceFilter<float> *, 3> covariances = (*Moments::getInstance()->motion).covariances;;

    std::vector<long> x_vals {1, 2, 3};
    std::vector<long> y_vals {4, 5, 6};

    for (size_t i = 0; i < 3; i++) {
        means[static_cast<int>(pos_val::X)]->onlineAverage(x_vals[i]);
        means[static_cast<int>(pos_val::Y)]->onlineAverage(y_vals[i]);
        var[static_cast<int>(pos_val::X)]->onlineVariance(x_vals[i], means[static_cast<int>(pos_val::X)]->getFilteredValue());
        var[static_cast<int>(pos_val::Y)]->onlineVariance(y_vals[i], means[static_cast<int>(pos_val::Y)]->getFilteredValue());
        covariances[0]->onlineCovariance(x_vals[i], y_vals[i],
                means[static_cast<int>(pos_val::X)]->getFilteredValue(), means[static_cast<int>(pos_val::Y)]->getFilteredValue());
    }
    std::cout << "Mean Filter: (" <<
        ((means[static_cast<int>(pos_val::X)]->getFilteredValue() == 2 && means[static_cast<int>(pos_val::Y)]->getFilteredValue() == 5) ? "PASS" : "FAIL") <<
        ")" << std::endl;
    std::cout << "Variance Filter (" <<
        ((var[static_cast<int>(pos_val::X)]->getFilteredVariance() > 0.6 && var[static_cast<int>(pos_val::X)]->getFilteredVariance() < 0.7) ? "PASS" : "FAIL") <<
        ")" << std::endl;
    std::cout << "Covariance Filter (" <<
        ((covariances[0]->getFilteredCovariance() > 0.6 && covariances[0]->getFilteredCovariance() < 0.7) ? "PASS" : "FAIL") <<
        ")" << std::endl;
}

void testEquations() {
    // Origin to Point
    RAY ray {.range = 5, .angle = 0.6435};
    POSE pose{2, 1, 0};
    POSE pt = Equations::getInstance()->originToPoint(ray, pose);

    std::cout << "Origin to pt. (" << ((abs(pt.x - 5) < 0.1 && abs(pt.y - 5) < 5) ? "PASS" : "FAIL") << ")" << std::endl;

    // Wrapping theta
    double largeRad =  M_PI + M_PI_2;
    double smallRad = M_PI_2;
    float largeWrap = Equations::getInstance()->wrapTheta(largeRad);
    float smallWrap = Equations::getInstance()->wrapTheta(smallRad);
    std::cout << "Wrapping (" << ((fabs(largeWrap) - 1.58) < 0.1 && (M_PI_2 - fabs(smallWrap) < 0.1) ? "PASS" : "FAIL") << ")" << std::endl;

    // Normalize Value
    float norm = Equations::getInstance()->normalizeValue(3, 0, 5);
    std::cout << "Norm (" << (fabs(norm - 0.6) < 0.1 ? "PASS" : "FAIL") << ")" << std::endl;

    // Centroid
    std::array<POSE, 3> pairs{
            POSE{0, 0, 0},
            POSE{3, 0, 0},
            POSE{3, 4, 0}
    };

    LOCATION result = Equations::getInstance()->centroid(pairs);
    std::cout << "Centroid (" << ((result.x == 2 && fabs(result.y - 1.3) < 0.1) ? "PASS" : "FAIL") << ")" <<std::endl;

    // Cantor
    std::cout << "Cantor (" <<
        ((Equations::getInstance()->cantor(1, -3) == Equations::getInstance()->cantor(1.00001, -3)) ?
        "FAIL" : "PASS") << ")" << std::endl;

    // Distance between two points
    POSE temp = {.x = -1, .y = -7, 0};
    POSE other = {.x = 2, .y = -3, 0};
    std::cout << "Distance between pts (" <<
        ((Equations::getInstance()->distBetweenPts(temp, other) == 5) ? "PASS" : "FAIL") <<
        ")" << std::endl;
}

void testRedBlackTree() {
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
        FEATURE{},
        FEATURE{},
        FEATURE{}
    };
    sampleFeatures[0].pose.x = 0.3;
    sampleFeatures[0].pose.y = 1.57;
    sampleFeatures[0].incidentRay.angle = 3.14;

    for (int i = 0; i < sampleClassifiers.size(); i++) {
        sampleClassifiers[i].signature = (float) sampleSignatures[i];
    }

    // Create tree
    for (auto classifier : sampleClassifiers) {
        ActiveRovers::getInstance()->getRoverByName("achilles")
        .getLocalMap()->addToTree(classifier, sampleFeatures);
    }

    // Check if tree is balanced
    std::cout << "Root : " << ActiveRovers::getInstance()->getRoverByName("achilles").getLocalMap()->getRoot() << std::endl;
    ActiveRovers::getInstance()->getRoverByName("achilles").getLocalMap()->printTree(
            new NODE_PTR{
                .valid = true,
                .node_ptr = *(ActiveRovers::getInstance()->getRoverByName("achilles").getLocalMap()->getRoot())}
                , 0);

    // Check ML classifier (looking for like features)
    uint16_t index = 0;
    (ActiveRovers::getInstance()->getRoverByName("achilles").getLocalMap())->findMLClassifier(dummyClassifier, index);
    std::cout << "Find Classifier (" << ((index == 6) ? "PASS" : "FAIL") << ")" << std::endl;

    // Get features from node
    std::array<FEATURE, 3> featuresToPop {
        FEATURE(),
        FEATURE(),
        FEATURE(),
    };

    // Testing whether features are passed by reference
    ActiveRovers::getInstance()->getRoverByName("achilles").getLocalMap()->getFeaturesFromNode(featuresToPop, index);
    float testVal = featuresToPop[0].pose.x;
    featuresToPop[0].pose.x = -1.2;
    std::cout << "Get features from node (" <<
        ((testVal == sampleFeatures[0].pose.x && sampleFeatures[0].pose.x != -1.2) ?
        "PASS" : "FAIL") << ")" << std::endl;

    // Reset Tree
    ActiveRovers::getInstance()->getRoverByName("achilles").getLocalMap()->resetTree();
    std::cout << "Reset Tree ("  << ((*ActiveRovers::getInstance()->getRoverByName("achilles").getLocalMap()->getRoot()) ? "FAIL" : "PASS") << ")" << std::endl;
}

void testDetections() {
    std::array<SONAR, 3> sampleSonar {
        SONAR{.id = sonar_id::LEFT, .observedRange = -1},
        SONAR{.id = sonar_id::CENTER, .observedRange = 1},
        SONAR{.id = sonar_id ::RIGHT, .observedRange = 1}
    };

    ActiveRovers::getInstance()->getRoverByName("achilles").getDetections()->MLIncidentRay(sampleSonar);
    std::cout << "Detection : " << ActiveRovers::getInstance()->getRoverByName("achilles").getDetections()->hasIncidentRay() << std::endl;
    RAY *ray = ActiveRovers::getInstance()->getRoverByName("achilles").getDetections()->getIncidentRay();
}

void testMatrix() {
    Matrix<float> matrix {
            {1, 2, 3},
            {4, 5, 6}
    };
    std::cout << "Matrix Template (" <<
        ((matrix[0][0] == 1) ? "PASS" : "FAIL") << ")" << std::endl;
    matrix.print();

    Matrix<float> idMat {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    };

    Matrix<float> testID(3, 3); testID.identity();
    std::cout << "Matrix Identity (" << (testID == idMat ? "PASS" : "FAIL") << ")" << std::endl;

    matrix.transpose();
    std::cout << "Matrix Transpose (" <<
        ((matrix[0][1] == 4 && matrix[2][0] == 3) ? "PASS" : "FAIL") << ")" << std::endl;
    matrix.print();


    std::cout << "Zero Matrix" <<std::endl;
    matrix.zeroMatrix();
    matrix.print();

    // Testing inverting matrices (also determinant)
    std::cout << "Original Mat: " << std::endl;
    Matrix<float> toInv{
            {3, 0, 2},
            {2, 0, -2},
            {0, 1, 1}
    };

    toInv.invert();

    Matrix<float> inverse {
        {0.2, 0.2, 0},
        {-0.2, 0.3, 1},
        {0.2, -0.3, 0}
    };

    toInv.print();

    std::cout << "Inverse (" <<
              ((toInv == inverse) ? "PASS" : "FAIL") << ")" << std::endl;

}

void testMatrixMan() {
    Matrix<float> matrix{
            {1, 0, 2},
            {0, 5, 0},
            {3, 0, 4}
    };

    Matrix<float> scalarTest{
            {3, 0, 6},
            {0, 15, 0},
            {9, 0, 12}
    };

    matrix *= 3;
    std::cout << "Matrix Scalar (" << ((matrix == scalarTest) ? "PASS" : "FAIL") << ")" << std::endl;

    // Testing adding matrices
    Matrix<float> first {
            {3, 0, 6},
            {0, 15, 0},
            {9, 0, 12}
    };
    Matrix<float> toAdd{
            {-3, 1, -6},
            {1, 15, 1},
            {-9, 1, -12}
    };

    Matrix<float> addTest{
            {0, 1, 0},
            {1, 30, 1},
            {0, 1, 0}
    };

    first = first + toAdd;
    std::cout << "Matrix Addition (" << ((first == addTest) ? "PASS" : "FAIL") << ")" << std::endl;

    // Testing subtracting matrices
    Matrix<float> mat_a{
        {1, -1, 2},
        {0.15, 0, 0},
        {1, 0, 1}
    };
    Matrix<float> mat_b{
        {1, 3, 5},
        {0.1, 0.7, -1},
        {(float) -0.0481578, 0, 0}
    };
    Matrix<float> resSub{
        {0, -4, -3},
        {0.05, -0.7, 1},
        {(float) 1.04816, 0, 1}
    };

    mat_a = mat_a - mat_b;
    std::cout << "Matrix Subtraction (" << ((mat_a == resSub) ? "PASS" : "FAIL") << ")" << std::endl;

    // Testing multiplying matrices
    Matrix<float> aMatrix {
            {1, -1.3, 2},
            {0, 3, 0},
            {4, 0, 5},
            {7.8, 6, 0}
    };

    Matrix<float> bMatrix {
            {7, 0},
            {8, -1},
            {9, 0}
    };

    Matrix<float> testMat {
            {14.6, 1.3},
            {24, -3},
            {73, 0},
            {102.6, -6}
    };

    Matrix<float> resMult = aMatrix * bMatrix;
    std::cout << "Matrix Multiplication (" << ((resMult == testMat) ? "PASS" : "FAIL") << ")" << std::endl;

    Matrix<float> mat_1 {
            {1, 2},
            {0, 4}
    };

    Matrix<float> mat_2 {
            {-2.3, 0},
            {5, 0}
    };
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
     * Verify location at each vertex.
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
    clock_t time = clock();
    for (VELOCITY velocity : control) {
        for (int i = 0; i < 10; i++) {
            seif->motionUpdate(velocity);
            seif->stateEstimateUpdate();
            seif->measurementUpdate(rays[j]);
            if (rays[j].range != lim_float::min() && rays[j].angle != lim_float::min()) {
                float x = rays[j].range * sin(rays[j].angle);
                float y = rays[j].range * cos(rays[j].angle);
                float yp = y - velocity.linear;
                float phi = atan2(x, yp);
                float Rp = (float) sqrt(pow(x, 2) + pow(yp, 2));

                rays[j].range = (phi > 1.74 / 2) ? lim_float::min() : Rp;
                rays[j].angle = (phi > 1.74 / 2) ? lim_float::min() : phi;
            }
            seif->sparsification();
        }
//        j++;

        // Uncomment to see location at critical pts.
        static bool isDriving = false;
        std::cout << ((isDriving = !isDriving) ? "DRIVING -> " : "TURNING -> ");
        rPose = seif->getRoverPose();
        std::cout << rPose.x << ", " << rPose.y << ", " << rPose.theta << std::endl;
    }

    long totalCycles = 16 * 10;
    std::cout << "Average time per cycle: " << ((float) (clock() - time) / CLOCKS_PER_SEC) / totalCycles << std::endl;

    rPose = seif->getRoverPose();
    std::cout << "Final Pose: " << rPose.x << ", " << rPose.y << ", " << rPose.theta << std::endl;

    if (fabs(rPose.theta) < 0.05) {
        std::cout << "SEIF (" << ((std::fabs(rPose.x) < 0.05 && std::fabs(rPose.y) < 0.05 && 1) ? "PASS" : "FAIL")
                  << ")" << std::endl;
    } else {
        std::cout << "SEIF (" << ((std::fabs(rPose.x) < 0.05 && std::fabs(rPose.y) < 0.05 && 0) ? "PASS" : "FAIL")
                  << ")" << std::endl;
    }
}

//void testFeatureSet() {
//    RAY incRay{.range = 2, .angle = (float) M_PI_2};
//    std::array<POSE, 3> rPoses {
//        POSE{.x = -2, .y = 5, .theta = (float) -M_PI_2},
//        POSE{.x = -1.57, .y = -1.57, .theta = -M_PI_4},
//        POSE{.x = 5, .y = -2, .theta = 0}
//    };
//
//    std::array<FEATURE, 7> features {
//        FEATURE{
//                .incidentRay = incRay,
//                .pose = Equations::getInstance()->originToPoint(incRay, rPoses[0], true)
//        },
//        FEATURE {
//                .incidentRay = incRay,
//                .pose = Equations::getInstance()->originToPoint(incRay, rPoses[1], true)
//        },
//        FEATURE {
//                .incidentRay = incRay,
//                .pose = Equations::getInstance()->originToPoint(incRay, rPoses[2], true)
//        },
//        FEATURE {
//                .incidentRay = incRay,
//                .pose = Equations::getInstance()->originToPoint(incRay, rPoses[0], true)
//        },
//        FEATURE {
//                .incidentRay = incRay,
//                .pose = Equations::getInstance()->originToPoint(incRay, rPoses[1], true)
//        },
//        FEATURE {
//                .incidentRay = incRay,
//                .pose = Equations::getInstance()->originToPoint(incRay, rPoses[2], true)
//        },
//        FEATURE {
//                .incidentRay = incRay,
//                .pose = Equations::getInstance()->originToPoint(incRay, rPoses[2], true)
//        },
//    };
//
//    for (int i = 0; i < 7; i++) {
//        FeatureSet::getInstance()->addToSet(features[i], rPoses[i]);
//        if (FeatureSet::getInstance()->readyToPublish()) {
//            auto FS = FeatureSet::getInstance()->publishSet();
//            publishFeatureSet(get<0>(FS), get<1>(FS));
//            ActiveRovers::getInstance()->getRoverByName(roverName).integrateLocalFS(get<0>(FS), get<1>(FS));
//        }
//    }
//}

void testEnv() {

}

