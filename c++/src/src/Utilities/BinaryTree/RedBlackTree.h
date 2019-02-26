//
// Created by Mason U'Ren on 2019-02-25.
//

#ifndef MULTIAGENTSLAM_REDBLACKTREE_H
#define MULTIAGENTSLAM_REDBLACKTREE_H

#include <vector>

#include "Node.h"

#define PROB_MIN 0.7

class RedBlackTree {
public:
    explicit RedBlackTree(unsigned long maxFeatures) :
        availableIndexPtr(0),
        root(0),
        tree(new std::vector<Node>(maxFeatures)) {}
    ~RedBlackTree() = default;

    long findMLClassifier(const CLASSIFIER &classifier);
    void addToTree(const CLASSIFIER &classifier, const std::array<FEATURE, 3> &features);
    void resetTree();
    void getFeaturesFromNode(std::array<FEATURE, 3> *featuresToPopulate, long nodeIndexPtr);

private:
    bool areaLikelihood(const float &area);
    bool orientationLikelihood(const float &orient);
    int singleRotation(const int &nodeIndexPtr, const dir &direction);
    int doubleRotation(const int &nodeIndexPtr, const dir &direction);
    bool isRed(const node_color &color);
    void incrimentPtr();
    void changeRootPtr(const int &index);

    long availableIndexPtr;
    long root;
    std::shared_ptr<std::vector<Node>> tree;
};


#endif //MULTIAGENTSLAM_REDBLACKTREE_H
