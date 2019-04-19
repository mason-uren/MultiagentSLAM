#include "Adapter.h"

using json = nlohmann::json;
using lim_float = std::numeric_limits<float>;

using std::unique_ptr;
using std::make_unique;
using std::string;
using std::get;

void Adapter::loadDefaultConfig() {
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

void Adapter::jsonInitialize() {
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
        for (int rover_id = 0; rover_id < slamConfig.numberOfRovers; rover_id++) {
            ROVER_CONFIG roverConfig = slamConfig.rovers[rover_id];
            if (roverConfig.valid) {
                roverConfig.ID = rover_id;
//                Rover *rover = dynamic_cast<Rover *>(RoverFactory::create(&roverConfig));
                Rover *rover = new Rover(&roverConfig);
                DETECTION_CONFIG detectionConfig = slamConfig.detectionConfig;
                SEIF_CONFIG seifConfig = slamConfig.seifConfig;
                LOCAL_MAP_CONFIG localMapConfig = slamConfig.localMapConfig;

                if (published_name == roverConfig.name &&
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

void Adapter::kinematicHandler(const POSE &pose, const VELOCITY &vel) {
    SlamAdapter::getInstance()->updateKinematics(roverName, pose, vel);
}

void Adapter::sonarHandler(const std::array<SONAR, 3> &sonar) {
    SlamAdapter::getInstance()->updateDetections(roverName, sonar);
}

void Adapter::slamHandler() {
    SlamAdapter::getInstance()->slamUpdate(roverName);
    publishBelief(*ActiveRovers::getInstance()->getRoverByName(roverName).getCurrentPose(),
                  ActiveRovers::getInstance()->getRoverByName(roverName).getConfidence());

}


// TODO: auxilaryRover Handler
void Adapter::aRH() { // `auxilaryBeliefs`
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
void Adapter::fsH() { // `auxilaryFeatureSets`
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
void Adapter::tH() {
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
void Adapter::publishFeatureSet(const std::array<FEATURE, FEATURE_LIMIT> &featureSet, const CLASSIFIER &classifer) {
    std::cout << "GLOBAL PUBLISHER FEATURE SET" << std::endl;
    if (FeatureSet::getInstance()->readyToPublish()) {
        auto FS = FeatureSet::getInstance()->publishSet();
        publishFeatureSet(get<0>(FS), get<1>(FS));
        ActiveRovers::getInstance()->getRoverByName(roverName).integrateLocalFS(get<0>(FS), get<1>(FS));
    }
}

void Adapter::publishBelief(const POSE &pose, const float &confi) {
    std::cout << "GLOBAL PUBLISHED BELIEF" << std::endl;
}

// TODO ROS stuff
void Adapter::publishTransformation(const POSE &trans, const string &rID) {
    std::cout << "GLOBAL PUBLISHER TRANSLATION" << std::endl;
    if (ActiveRovers::getInstance()->getRoverByName(roverName).readyToPublish()) {
        // Publish new transformation
    }
}