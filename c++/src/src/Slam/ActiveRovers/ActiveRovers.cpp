//
// Created by Mason U'Ren on 2019-02-13.
//

#include "ActiveRovers.h"

void ActiveRovers::addRover(Rover &rover) {
    this->active->insert(std::make_pair(rover.getName(), &rover));
}

Rover& ActiveRovers::getRoverByName(const std::string &name) {
    try {
        return *this->active->at(name);

    }
    catch (const std::out_of_range &error) {
        std::cerr << "Rover name not found : " << error.what() << std::endl;
        exit(EXIT_FAILURE);
    }
}

Rover& ActiveRovers::getRoverByID(const int &id) {
    try {
        std::string name;
        for (auto &roverObj : *this->active) {
            if (roverObj.second->getID() == id) {
                name = roverObj.second->getName();
                break;
            }
        };
        return this->getRoverByName(name);
    } catch (const std::out_of_range &error) {
        std::cerr << "Rover id not found : " << error.what() << std::endl;
        exit(EXIT_FAILURE);
    }
}

std::unordered_map<std::string, Rover *> * ActiveRovers::getActiveRovers() {
    return &(*this->active);
}

