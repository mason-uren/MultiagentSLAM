//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_ACTIVEROVERS_H
#define C_ACTIVEROVERS_H

#include <unordered_map>
#include <SLAMConfigIn.h>
#include <RoverInterface.h>
#include "../../Agent/Rover/Rover.h"

class ActiveRovers {
public:
    static ActiveRovers *getInstance() {
        static ActiveRovers instance;
        return &instance;
    }

    void addRover(Rover &rover);
    bool getRoverByName(const std::string &name, Rover &rover);
    bool getRoverByID(const int &id, Rover &rover);
    std::unordered_map<std::string, Rover *> *getActiveRovers();

private:
    ActiveRovers() :
            active(new std::unordered_map<std::string, Rover *>()) {}
    ActiveRovers(ActiveRovers const&);
    void operator=(ActiveRovers const&);

    std::shared_ptr<std::unordered_map<std::string, Rover *>> active;
};


#endif //C_ACTIVEROVERS_H
