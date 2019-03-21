//
// Created by Mason U'Ren on 2019-03-10.
//

#ifndef MULTIAGENTSLAM_MATRIXMANIPULATOR_H
#define MULTIAGENTSLAM_MATRIXMANIPULATOR_H

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>

#include <Matrix.h>
#include <SharedMemoryStructs.h>
//#include <Tools.h>

#include "../../../../src/Utilities/Equations/Equations.h"

class MatrixManipulator {
public:
    static MatrixManipulator *getInstance() {
        static MatrixManipulator instance;
        return &instance;
    }

    template <typename T, typename S> Matrix<T> quadratic(const Matrix<T> *outerMat, const Matrix<S> *innerMat);
    template <typename T, typename S> Matrix<T> multiply(const Matrix<T> *matrixA, const Matrix<S> *matrixB);
    template <typename T, typename S> void subtract(Matrix<T> *result, const Matrix<S> *matrix);
    template <typename T, typename S> void add(Matrix<T> *result, const Matrix<S> *matrix);

    template <typename T, typename S> void scalarMult(Matrix<T> *matrix, const T &scalar);

private:
    MatrixManipulator() = default;
    MatrixManipulator(MatrixManipulator const &);
    void operator=(MatrixManipulator const &);

    template <typename T> bool sameShape(const Matrix<T> *matrix, const Matrix<T> *other);
    template <typename T> bool canMultiply(const Matrix<T> *matrixA, const Matrix<T> *matrixB);
};

// IMPORTANT: never pass outer matrix in transpose form.
template<typename T, typename S>
Matrix<T> MatrixManipulator::quadratic(const Matrix<T> *outerMat, const Matrix<S> *innerMat) {
    Matrix<T> transpose = *outerMat; transpose.transpose();
    Matrix<T> leftHS;

    if (outerMat->numCols() == innerMat->numRows()) {
        leftHS = multiply<T, S>(outerMat, innerMat);
        return multiply<T, S>(&leftHS, &transpose);
    }
    else if (outerMat->numRows() == innerMat->numRows()) {
        leftHS = multiply<T, S>(&transpose, innerMat);
        return multiply<T, S>(&leftHS, outerMat);
    }

    exit(EXIT_FAILURE);
}

template<typename T, typename S>
Matrix<T> MatrixManipulator::multiply(const Matrix<T> *matrixA, const Matrix<S> *matrixB) {
    if (!canMultiply(matrixA, matrixB)) {
        return Matrix<T>{};
    }
    return *matrixA * *matrixB;
}

template<typename T, typename S>
void MatrixManipulator::subtract(Matrix<T> *result, const Matrix<S> *matrix) {
    if (sameShape(result, matrix)) {
        *result -= *matrix;
    }
}

template<typename T, typename S>
void MatrixManipulator::add(Matrix<T> *result, const Matrix<S> *matrix) {
    if (sameShape(result, matrix)) {
        *result += *matrix;
    }
}

template<typename T, typename S>
void MatrixManipulator::scalarMult(Matrix<T> *matrix, const T &scalar) {
    *matrix *= *scalar;
}

template<typename T>
bool MatrixManipulator::sameShape(const Matrix<T> *matrix, const Matrix<T> *other) {
    return matrix->numRows() == other->numRows() && matrix->numCols() == other->numCols();
}

template<typename T>
bool MatrixManipulator::canMultiply(const Matrix<T> *matrixA, const Matrix<T> *matrixB) {
    return (matrixA && matrixB) && (matrixA->numCols() == matrixB->numRows());
}

#endif //MULTIAGENTSLAM_MATRIXMANIPULATOR_H
