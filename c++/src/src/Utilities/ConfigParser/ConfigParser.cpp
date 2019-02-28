//
// Created by Mason U'Ren on 2019-02-14.
//

#include "ConfigParser.h"

// for convenience
using json = nlohmann::json;

void ConfigParser::parseConfig(SYS_CONFIG_IN *in, json *dataPtr) {
    std::hash<nlohmann::json> hasher;
    json data = *dataPtr;

    JSON_CONFIG *config = &in->config;
    SLAM_CONFIG *slamConfig;
    ROVER_CONFIG *roversConfig;
    SEIF_CONFIG *seifConfig;
    DETECTION_CONFIG *detectionConfig;
    LOCAL_MAP_CONFIG *localMapConfig;

    config->hash = hasher(data);

    /**
     * TODO Doesn't necessary warrent returning too but 'config' structures might change.
     */

    try {
        slamConfig = &config->slamConfig;
        if (!data["slamConfig"].is_null()) {
            slamConfig->valid = true;
            slamConfig->numberOfRovers = data["slamConfig"]["numberOfRovers"].get<int>();
            slamConfig->filterSize = data["slamConfig"]["filterSize"].get<int>();
            slamConfig->maxFeatures = data["slamConfig"]["maxFeatures"].get<unsigned long>();

            for (unsigned int rover_id = 0; rover_id < slamConfig->numberOfRovers; rover_id++) {
                roversConfig = &(slamConfig->rovers[rover_id]);
                if (!data["slamConfig"]["rovers"][rover_id].is_null()) {
                    roversConfig->valid = true;
//                    roversConfig->ID = data["Slam"]["Rovers"][rover_id]["ID"].get<int>();
                    roversConfig->name = data["slamConfig"]["rovers"][rover_id]["name"].get<std::string>();
                    roversConfig->live = data["slamConfig"]["rovers"][rover_id]["live"].get<bool>();
                    if (roversConfig->live) {
                        detectionConfig = &(roversConfig[rover_id].detectionConfig);
                        if (!data["slamConfig"]["rovers"][rover_id]["detection"].is_null()) {
                            detectionConfig->valid = true;
                            detectionConfig->highDetectionBoundaryInM =
                                    data["slamConfig"]["rovers"][rover_id]["detection"]["highDetectionBoundaryInM"].get<float>();
                            detectionConfig->sonarCoverageInRad =
                                    data["slamConfig"]["rovers"][rover_id]["detection"]["sonarCoverageInRad"].get<float>();
                            detectionConfig->sonarRangeInM =
                                    data["slamConfig"]["rovers"][rover_id]["detection"]["sonarRangeInM"].get<float>();
                        }
                        else {
                            detectionConfig->valid = false;
                        }
                        seifConfig = &(roversConfig[rover_id].seifConfig);
                        if (!data["slamConfig"]["rovers"][rover_id]["seif"].is_null()) {
                            seifConfig->valid = true;
                            seifConfig->maxActiveFeatures =
                                    data["slamConfig"]["rovers"][rover_id]["seif"]["maxActiveFeatures"].get<int>();
                            seifConfig->maxFeatures = slamConfig->maxFeatures;
//                            seifConfig->maxFeatures =
//                                    data["slamConfig"]["rovers"][rover_id]["seif"]["maxFeatures"].get<long>();
                        }
                        else {
                            seifConfig->valid = false;
                        }
                        localMapConfig = &(roversConfig[rover_id].localMapConfig);
                        if (!data["slamConfig"]["rovers"][rover_id]["localMap"].is_null()) {
                            localMapConfig->valid = true;
                            localMapConfig->featureSetML = data["slamConfig"]["rovers"][rover_id]["localMap"]["featureSetML"].get<float>();
                            localMapConfig->maxFeatures = slamConfig->maxFeatures;
                        }
                        else {
                            localMapConfig->valid = false;
                        }
                    }
                }
                else {
                    roversConfig->valid = false;
                }
            }
        }
        else  {
            slamConfig->valid = false;
        }

    } catch (nlohmann::json::parse_error &e) {
        std::cout << e.what() << std::endl;
    }

}

bool ConfigParser::loadJSONFromFile(const std::string &filePath, json *dataPtr) {
    std::ifstream configFile(filePath);
    if (configFile.is_open()) {
        try {
            configFile >> *dataPtr;
            configFile.close();
            return true;
        } catch (nlohmann::detail::parse_error &e) {
            std::cout << e.what() << std::endl;
        }
    }
    return false;
}


