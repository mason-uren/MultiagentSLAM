//
// Created by Mason U'Ren on 2019-03-11.
//

#ifndef MULTIAGENTSLAM_MATRIX_H
#define MULTIAGENTSLAM_MATRIX_H

#include <iostream>
#include <algorithm>

template <class T>
class Matrix {
public:
    Matrix(const unsigned long &rows, const unsigned long &cols = 1) :
        data(new std::vector<T>(std::max(rows, (u_long) 1) * std::max(cols, (u_long) 1))),
        nRows(rows),
        nCols(cols),
        trans(false)
    {}
    Matrix() = default;
    ~Matrix() = default;

    // TODO may need to build copy constructor

    // TODO may ?need to build string parser for 'getType' to elimate garbage characters

    bool operator==(Matrix &matrix);

    // CAUTION: does not conform to 2D matrix design
    // Should never be used outside of MatrixManipulator.h
    T &operator[](const unsigned long index);
    T &at(const unsigned long row, const unsigned long col) const;
    void transpose();
    unsigned long numRows() const;
    unsigned long numCols() const;
    void resetMatrix();

    // For testing purposes only
    void print();

private:
    std::shared_ptr<std::vector<T>> data;
    unsigned long nRows{};
    unsigned long nCols{};
    bool trans{};
};

template <typename T>
bool Matrix<T>::operator==(Matrix<T> &matrix) {
    for (auto index = 0; index < (nRows * nCols); index++) {
        unsigned long majorOrder = (trans) ? nRows : nCols;
        if ((*data)[index] != matrix.at(index / majorOrder, index % majorOrder)) {
            return false;
        }
    }
    return true;
}

// CAUTION: does not conform to 2D matrix design
// Should never be used outside of MatrixManipulator.h
template <typename T>
T &Matrix<T>::operator[](const unsigned long index) {
    if (index >= nRows * nCols) {
        std::cout << "Warning: dangerous operator" << std::endl;
        std::cout << "Matrix : index out of bounds... Exiting" << std::endl;
        exit(0);
    }
    return (*data)[index];
}

template <typename T>
T &Matrix<T>::at(const unsigned long row, const unsigned long col) const {
    if (row >= nRows) {
        std::cout << "Matrix : row indice out of bounds... Exiting" << std::endl;
        exit(EXIT_FAILURE);
    } else if (col >= nCols) {
        std::cout << "Matrix : col indice out of bounds... Exiting" << std::endl; ;
        exit(EXIT_FAILURE);
    }
    return (*data)[(!trans) ? row * nCols + col : col * nRows + row];
}

template <typename T>
void Matrix<T>::transpose() {
    trans = !trans;
    std::swap(nRows, nCols);
}

template <typename T>
unsigned long Matrix<T>::numRows() const {
    return nRows;
}

template <typename T>
unsigned long Matrix<T>::numCols() const {
    return nCols;
}

template <typename T>
void Matrix<T>::print() {
    for (auto index = 0; index < (nRows * nCols); index++) {
        ((!index || index % nCols) ? printf("%f ", (*data)[index]) : printf("\n%f ", (*data)[index]));
    }
    printf("\n");
}

template<class T>
void Matrix<T>::resetMatrix() {
    (*data) = std::vector<T>(nRows * nCols);
    nRows = 0, nCols = 0;
}

#endif //MULTIAGENTSLAM_MATRIX_H
