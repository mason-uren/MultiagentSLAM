//
// Created by Mason U'Ren on 2019-02-25.
//

#include "RedBlackTree.h"

// Should only enter when tree is empty
//    if (!availableIndexPtr && !root) {
//        Node *node = &(*this->tree)[root];
//        node->classifier->area = classifier.area;
//        node->classifier->orientation = classifier.orientation;
//        node->classifier->signature = classifier.signature;
//        node->isSet = true;
//        this->incrimentPtr();
//    }
//    else {
//        int head = 0;
//        // Grandparent, Iterator, and parent
//        Node *g, *p, *q;
//        dir direction = dir::LEFT;
//        int last;
//
//        /* Setup helpers */
//        g = p = nullptr;
//        q = &(*this->tree)[root];
//
//        while (true) {
//            if (!q->isSet) {
//
//            }
//        }
//    }
long RedBlackTree::findMLClassifier(const CLASSIFIER &classifier) {
    if (!availableIndexPtr && !root) {
        return availableIndexPtr;
    }

    Node *node = &(*this->tree)[root];
    long index = root;

    return index;
}

void RedBlackTree::addToTree(const CLASSIFIER &classifier, const std::array<FEATURE, 3> &features) {

}

void RedBlackTree::resetTree() {

}

void RedBlackTree::getFeaturesFromNode(std::array<FEATURE, 3> *featuresToPopulate, long nodeIndexPtr) {

}

bool RedBlackTree::areaLikelihood(const float &area) {
    return false;
}

bool RedBlackTree::orientationLikelihood(const float &orient) {
    return false;
}

int RedBlackTree::singleRotation(const int &nodeIndexPtr, const dir &direction) {
    return 0;
}

int RedBlackTree::doubleRotation(const int &nodeIndexPtr, const dir &direction) {
    return 0;
}

bool RedBlackTree::isRed(const node_color &color) {
    return false;
}

void RedBlackTree::incrimentPtr() {
    this->availableIndexPtr++;
}

void RedBlackTree::changeRootPtr(const int &index) {

}
