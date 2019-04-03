//
// Created by Mason U'Ren on 2019-02-13.
//

#include "ActiveRovers.h"

void ActiveRovers::addRover(Rover &rover) {
    this->active->insert(std::make_pair(rover.getName(), &rover));
}

bool ActiveRovers::getRoverByName(const std::string &name, Rover &rover) {
    try {
        rover = *this->active->at(name);
        return true;
    }
    catch (const std::out_of_range &error) {
        std::cerr << "Rover name not found : " << error.what() << std::endl;
    }
    return false;
}

bool ActiveRovers::getRoverByID(const int &id, Rover &rover) {
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
        std::cerr << "Rover CORRESPONDENCE not found : " << error.what() << std::endl;
    }
    return false;
}

std::unordered_map<std::string, Rover *> * ActiveRovers::getActiveRovers() {
    return &(*this->active);
}

