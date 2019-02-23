//
// Created by Mason U'Ren on 2019-02-13.
//

#include "ActiveRovers.h"

void ActiveRovers::addRover(RoverInterface &rover) {
    this->active->insert(std::make_pair(rover.getName(), &rover));
}

bool ActiveRovers::getRoverByName(const std::string &name, Rover *rover) {
    try {
        rover = dynamic_cast<Rover *>(this->active->at(name));
        return false;
    }
    catch (const std::out_of_range &error) {
        std::cerr << "Rover name not found : " << error.what() << std::endl;
    }
    return false;
}

bool ActiveRovers::getRoverByID(const int &id, Rover *rover) {
    try {
        std::string name;
        for (auto &roverObj : *this->active) {
            if (roverObj.second->getID() == id) {
                name = roverObj.second->getName();
                break;
            }
        };
        return this->getRoverByName(name, rover);
    } catch (const std::out_of_range &error) {
        std::cerr << "Rover ID not found : " << error.what() << std::endl;
    }
    return false;
}

std::unordered_map<std::string, RoverInterface *> * ActiveRovers::getActiveRovers() {
    return &(*this->active);
}

