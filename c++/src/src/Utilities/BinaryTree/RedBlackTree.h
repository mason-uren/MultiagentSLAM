//
// Created by Mason U'Ren on 2019-02-25.
//

#ifndef MULTIAGENTSLAM_REDBLACKTREE_H
#define MULTIAGENTSLAM_REDBLACKTREE_H

#include <vector>
#include <algorithm>
#include <limits>
#include <cstddef>
#include <functional>
#include <SharedMemoryStructs.h>
#include <boost/math/distributions.hpp>
#include "../Equations/Equations.h"

#include "Node.h"

#define PDF_MAX 0.4 // Roughly

enum dir {
    LEFT = 0,
    RIGHT
};

class RedBlackTree {
public:
    explicit RedBlackTree(LOCAL_MAP_CONFIG *localMapConfig) :
        availableIndexPtr(0),
        root(0),
        tree(new std::vector<Node>(localMapConfig->maxFeatures)),
        cleanTree(new std::vector<Node>(localMapConfig->maxFeatures)),
        minProbability((float) (localMapConfig->featureSetML * PDF_MAX)) {}
    ~RedBlackTree() = default;

    bool findMLClassifier(const CLASSIFIER &classifier, unsigned long &index);
    void addToTree(const CLASSIFIER &classifier, const std::array<FEATURE, 3> &features);
    void resetTree();
    void getFeaturesFromNode(std::array<FEATURE, 3> &featuresToPopulate,
            const unsigned long &nodeIndexPtr);

    // For testing purposes only!
    void printTree(NODE_PTR *root, int  level);
    unsigned long *getRoot() {
        return &(root);
    }

private:
    float areaLikelihood(const float &area);
    float orientationLikelihood(const float &orient);
    unsigned long singleRotation(const unsigned long &nodeIndex, const dir &direction);
    unsigned long doubleRotation(const unsigned long &nodeIndex, const dir &direction);
    bool isRed(unsigned long *nodeIndex);
    bool isRed(Node *node);
    void incrimentPtr();
    void changeRootPtr(const unsigned long &index);
    void changeNodeColor(const unsigned long &nodeIndex, const node_color *desiredColor);
    void balanaceTree(const float &signature);
    Node *getNodeAt(const unsigned long *nodeIndex);
    dir changeDir(const dir &current);
    void rotateNodes(const dir &ofRotation, Node &parent, const unsigned long &index);
    void setChild(NODE_PTR *ptr, const unsigned long &newValue);
    bool assignChildTo(NODE_PTR &node_ptr, unsigned long *otherPtr);

    unsigned long availableIndexPtr;
    unsigned long root;
    std::shared_ptr<std::vector<Node>> tree;
    std::shared_ptr<std::vector<Node>> cleanTree;
    float minProbability;
};


#endif //MULTIAGENTSLAM_REDBLACKTREE_H
