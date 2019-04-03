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

#include "../../../../src/Utilities/Equations/Equations.h"

class MatrixManipulator {
public:
    static MatrixManipulator *getInstance() {
        static MatrixManipulator instance;
        return &instance;
    }

    template <typename T, typename S> Matrix<T> quadratic(const Matrix<T> *outerMat, const Matrix<S> *innerMat, const bool &frontTrans = false);
    template <typename T, typename S> Matrix<T> multiply(const Matrix<T> *matrixA, const Matrix<S> *matrixB);
    template <typename T, typename S> void subtract(Matrix<T> *result, const Matrix<S> *matrix);
    template <typename T, typename S> void add(Matrix<T> *result, const Matrix<S> *matrix);

    template <typename T> void scalarMult(Matrix<T> *matrix, const float &scalar);

private:
    MatrixManipulator() = default;
    MatrixManipulator(MatrixManipulator const &);
    void operator=(MatrixManipulator const &);

    template <typename T> bool sameShape(const Matrix<T> *matrix, const Matrix<T> *other);
    template <typename T> bool canMultiply(const Matrix<T> *matrixA, const Matrix<T> *matrixB);
};

// IMPORTANT: never pass matrices in transpose form
template<typename T, typename S>
Matrix<T> MatrixManipulator::quadratic(const Matrix<T> *outerMat, const Matrix<S> *innerMat, const bool &frontTrans) {
    Matrix<T> transpose = *outerMat; transpose.transpose();
    Matrix<T> leftHS;

    if (frontTrans) {
        leftHS = multiply<T, S>(&transpose, innerMat);
        return multiply<T, S>(&leftHS, outerMat);
    }
//    else if (outerMat->numRows() == innerMat->numRows()) {
    else {
        leftHS = multiply<T, S>(outerMat, innerMat);
        return multiply<T, S>(&leftHS, &transpose);
    }
//
//    exit(EXIT_FAILURE);
}

template<typename T, typename S>
Matrix<T> MatrixManipulator::multiply(const Matrix<T> *matrixA, const Matrix<S> *matrixB) {
    if (!canMultiply(matrixA, matrixB)) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << matrixA->numRows() << ", " << matrixA->numCols() << ")" << std::endl;
        std::cerr << "dims : B(" << matrixB->numRows() << ", " << matrixB->numCols() << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    return *matrixA * *matrixB;
}

template<typename T, typename S>
void MatrixManipulator::subtract(Matrix<T> *result, const Matrix<S> *matrix) {
    if (!sameShape(result, matrix)) {
        std::cerr << "Cannot multiply give matrices." << std::endl;
        std::cerr << "dims : A(" << result->numRows() << ", " << result->numCols() << ")" << std::endl;
        std::cerr << "dims : B(" << matrix->numRows() << ", " << matrix->numCols() << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    *result -= *matrix;
}

template<typename T, typename S>
void MatrixManipulator::add(Matrix<T> *result, const Matrix<S> *matrix) {
    if (!sameShape(result, matrix)) {
        std::cerr << "Cannot multiply give matrices." << std::endl;
        std::cerr << "dims : A(" << result->numRows() << ", " << result->numCols() << ")" << std::endl;
        std::cerr << "dims : B(" << matrix->numRows() << ", " << matrix->numCols() << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    *result += *matrix;
}

template<typename T>
void MatrixManipulator::scalarMult(Matrix<T> *matrix, const float &scalar) {
    *matrix *= scalar;
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
