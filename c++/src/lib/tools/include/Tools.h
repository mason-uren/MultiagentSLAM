//
// Created by Mason U'Ren on 2019-03-14.
//

#ifndef MULTIAGENTSLAM_TOOLS_H
#define MULTIAGENTSLAM_TOOLS_H

#include <SharedMemoryStructs.h>

class Tools {
public:
    static Tools *getInstance() {
        static Tools instance;
        return &instance;
    }

    obj_type type(const std::string &id);

private:
    Tools() = default;
    Tools(Tools const &) = delete;
    void operator=(Tools const &) = delete;
};

obj_type Tools::type(const std::string &id) {
    if (id == "f" || id == "float") {
        return obj_type::FLOAT;
    }
    if (id == "l" || id == "long" || id == "m") {
        return obj_type::LONG;
    }
    if (id == "7ELEMENT") {
        return obj_type::OBJECT;
    }
    return obj_type::UNDEFINED;
}

#endif //MULTIAGENTSLAM_TOOLS_H
