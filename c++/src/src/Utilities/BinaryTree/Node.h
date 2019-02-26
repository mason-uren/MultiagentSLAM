//
// Created by Mason U'Ren on 2019-02-25.
//

#ifndef MULTIAGENTSLAM_NODE_H
#define MULTIAGENTSLAM_NODE_H

#include <iostream>
#include <array>
#include <SharedMemoryStructs.h>
#include <boost/math/distributions.hpp>

using namespace boost::math;

class Node {
public:
    Node() :
        classifier(new CLASSIFIER{.area = 0, .orientation = 0, .signature = 0}),
        featureSet(new std::array<FEATURE, 3>
                {
                    (FEATURE{.xRelative = 0, .yRelative = 0, .incidentRay = 0}),
                    (FEATURE{.xRelative = 0, .yRelative = 0, .incidentRay = 0}),
                    (FEATURE{.xRelative = 0, .yRelative = 0, .incidentRay = 0})
                }
        ),
        color(node_color::RED),
        isSet(false) {}
    ~Node() = default;

    std::shared_ptr<CLASSIFIER> classifier;
    std::shared_ptr<std::array<FEATURE, 3>> featureSet;
    node_color color;
    std::shared_ptr<int> parent;
    std::shared_ptr<int> leftChild;
    std::shared_ptr<int> rightChild;
    bool isSet;

    static std::shared_ptr<normal> distribution;
};

#endif //MULTIAGENTSLAM_NODE_H
