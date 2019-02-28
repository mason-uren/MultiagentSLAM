//
// Created by Mason U'Ren on 2019-02-25.
//

#ifndef MULTIAGENTSLAM_NODE_H
#define MULTIAGENTSLAM_NODE_H

#include <iostream>
#include <array>

using namespace boost::math;

typedef struct {
    bool valid;
    unsigned long node_ptr;
} NODE_PTR;

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
        location(new NODE_PTR{.valid = false, .node_ptr = 0}),
        leftChild(new NODE_PTR{.valid = false, .node_ptr = 0}),
        rightChild(new NODE_PTR{.valid = false, .node_ptr = 0})
        {}
    ~Node() = default;

    bool operator == (const Node &node) const {
        return classifier->signature == node.classifier->signature;
    }

    std::shared_ptr<CLASSIFIER> classifier;
    std::shared_ptr<std::array<FEATURE, 3>> featureSet;
    node_color color;
    std::shared_ptr<NODE_PTR> location;
    std::shared_ptr<NODE_PTR> leftChild;
    std::shared_ptr<NODE_PTR> rightChild;

    static const std::shared_ptr<normal> distribution;

    void saveLocationPtr(const unsigned long &index) {
        if (!location->valid) {
            location->valid = true;
            location->node_ptr = index;
        }
    }
};

#endif //MULTIAGENTSLAM_NODE_H
