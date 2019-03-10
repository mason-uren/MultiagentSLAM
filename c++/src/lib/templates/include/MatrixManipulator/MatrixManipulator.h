//
// Created by Mason U'Ren on 2019-03-10.
//

#ifndef MULTIAGENTSLAM_MATRIXMANIPULATOR_H
#define MULTIAGENTSLAM_MATRIXMANIPULATOR_H

#include <cmath>
#include <vector>
#include <algorithm>
#include <Matrix.h>
#include <SharedMemoryStructs.h>
#include <Tools.h>
#include <functional>
#include "../../../../src/Utilities/Equations/Equations.h"

//namespace inner {
//
//    struct Cell {
//
//        template <typename T>
//        static void ref(std::vector<float *> &ref, T * const cell) {
//            switch (cell->type) {
//                case descriptor::STATE_ESTIMATE:
//                    ref[pos_val::X] = &(cell->pose.x);
//                    ref[pos_val::Y] = &(cell->pose.y);
//                    ref[pos_val::THETA] = &(cell->pose.theta);
//                case descriptor::FOUND_FEATURE:
//                    ref[ray_val::RANGE] = &(cell->feature.incidentRay.range);
//                    ref[ray_val::ANGLE] = &(cell->feature.incidentRay.angle);
//                    ref[2] = nullptr;
//            }
//        }
//
//        template <typename T>
//        static void ref(std::vector<float *> &ref, float * const cell) {
//            ref[0] = cell;
//            ref[1] = nullptr;
//            ref[2] = nullptr;
//        }
//    };
//}

class MatrixManipulator {
public:
    static MatrixManipulator *getInstance() {
        static MatrixManipulator instance;
        return &instance;
    }

    template <typename T> void invert(Matrix<T> *matrix);
    template <typename T> long determinant(Matrix<T> *matrix, const unsigned long &order);
    template <typename T, typename S> void multiply(Matrix<T> *result, const Matrix<T> *matrixA, const Matrix<S> *matrixB);
    template <typename T, typename S> void subtract(Matrix<T> *result, const Matrix<S> *matrix);
    template <typename T, typename S> void add(Matrix<T> *result, const Matrix<S> *matrix);

private:
    MatrixManipulator() = default;
    MatrixManipulator(MatrixManipulator const &);
    void operator=(MatrixManipulator const &);


//    typedef float (*eval)(float &, const float &);
//    static float plus(float &a, const float &b);
//    static float minus(float &a, const float &b);
//    static float multi(float &a, const float &b);

//    template <typename T, typename S> void operations(T &result, S &cell, eval oper, T *ret_val = nullptr);
    template <typename T> void cofactor(const Matrix<T> *matrix, Matrix<T> *temp,
            const unsigned long &currRow, const unsigned long &currCol, const unsigned long &order);
    template <typename T> void adjoint(Matrix<T> *matrix, Matrix<T> *temp);
    template <typename T> bool isSquare(const Matrix<T> *matrix);
    template <typename T> bool sameShape(const Matrix<T> *matrix, const Matrix<T> *other);
    template <typename T> bool canMultiply(const Matrix<T> *result, const Matrix<T> *matrixA, const Matrix<T> *matrixB);
};

template<typename T>
void MatrixManipulator::invert(Matrix<T> *matrix) {
    unsigned long order = matrix->numRows();
    if (float det = determinant(matrix, order)) {
        unsigned long rows = matrix->numRows();
        unsigned long cols = matrix->numCols();

        Matrix<T> inverse(rows, cols);
        Matrix<T> adj(rows, cols);
        adjoint(matrix, &adj);

        // Find inverse using "inverse(A) = adj(A)/det(A)"
        for (unsigned long i = 0; i < rows; i++) {
            for (unsigned long j = 0; j < cols; j++) {
                inverse.at(i, j) = adj.at(i, j) / det;
            }
        }

        // Copy over inverse
        (*matrix) = inverse;
    }
}

template<typename T>
long MatrixManipulator::determinant(Matrix<T> *matrix, const unsigned long &order) {
    if (!isSquare(matrix)) {
            return 0;
    }

    long det = 0;
    if (order < 2) {
        return (*matrix).at(0, 0);
    }

    Matrix<T> tempMatrix(order, order);
    int sign = 1;

    for (unsigned long col = 0; col < order; col++) {
        cofactor(matrix, &tempMatrix, 0, col, order);
        det += sign * matrix->at(0, col) * determinant(&tempMatrix, (order - 1));
        sign = -sign;
    }

    return det;
}

template<typename T, typename S>
void MatrixManipulator::multiply(Matrix<T> *result, const Matrix<T> *matrixA, const Matrix<S> *matrixB) {
    if (!canMultiply(result, matrixA, matrixB)) {
        return;
    }
    unsigned long row = 0;
    unsigned long col = 0;
    unsigned long iter = 0;
    T ret_val = T();
    while (iter < matrixA->numCols()) {
        result->at(row, col) += (matrixA->at(row, iter) * matrixB->at(iter, col));
//        operations<T, S>(matrixA->at(row, iter), matrixB->at(iter, col), multi, &ret_val);
//        operations<T, S>(result->at(row, col), ret_val, plus);
        iter++;
        if (!(iter % matrixA->numCols())) {
            col++;
            iter = 0;
            if (!(col % result->numCols())) {
                row++;
                col = 0;
                if (!(row % matrixA->numRows())) {
                    break;
                }
            }
        }
    }
}

template<typename T, typename S>
void MatrixManipulator::subtract(Matrix<T> *result, const Matrix<S> *matrix) {
    if (sameShape(result, matrix)) {
        for (unsigned long row = 0; row < result->numRows(); row++) {
            for (unsigned long col = 0; col < result->numCols(); col++) {
                result->at(row, col) -= matrix->at(row, col);
//                operations<T, S>(result->at(row, col), matrix->at(row, col), minus);
            }
        }
    }
}

template<typename T, typename S>
void MatrixManipulator::add(Matrix<T> *result, const Matrix<S> *matrix) {
    if (sameShape(result, matrix)) {
        for (unsigned long row = 0; row < result->numRows(); row++) {
            for (unsigned long col = 0; col < result->numCols(); col++) {
                result->at(row, col) += matrix->at(row, col);
//                operations<T, S>(result->at(row, col), matrix->at(row, col), plus);
            }
        }
    }
}

/**
 * @fn operations
 * @brief Applies operation to provided matrices.
 *
 * The provided operation will be applied in the order of parameter position (ie. add(result + matrix)).
 * Result shall be stored within the result matrix.
 * @tparam T - #ELEMENT (primative type if #S is primative)
 * @tparam S - #ELEMENT or other #obj_type
 * @param result - the matrix to which the operation will be applied
 * @param matrix - the matrix that is being added.
 */
//template <typename T, typename S>
//void MatrixManipulator::operations(T &result, S &cell, eval oper, T *ret_val) {
//    std::vector<float*> rValues(3, 0);
//    std::vector<float*> cValues(3, 0);
//    std::vector<float*> returned(3, 0);
//    std::vector<float> temp(3, 0);
//    inner::Cell::ref<T>(rValues, &result);
//    inner::Cell::ref<S>(cValues, &cell);
//
//    if (ret_val) {
//        inner::Cell::ref<T>(returned, ret_val);
//    }
//
//    bool isFloat = (cValues[1] == nullptr);
//    for (size_t i = 0; i < rValues.size() && rValues[i]; i++) {
//        (isFloat) ? temp[i] = oper(*rValues[i], *cValues[0]) : temp[i] = oper(*rValues[i], *cValues[i]);
//        if (ret_val) {
//            *returned[i] = temp[i];
//        }
//    }
//}

template<typename T>
void MatrixManipulator::cofactor(const Matrix<T> *matrix, Matrix<T> *temp,
        const unsigned long &currRow, const unsigned long &currCol, const unsigned long &order) {
    if (!isSquare(matrix)) {
        return;
    }

    unsigned long i = 0, j = 0;
    for (unsigned long row = 0; row < order; row++) {
        for (unsigned long col = 0; col < order; col++) {
            if (!(row == currRow || col == currCol)) {
                temp->at(i, j++) = matrix->at(row, col);
                if (j == order - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

template<typename T>
void MatrixManipulator::adjoint(Matrix<T> *matrix, Matrix<T> *temp) {
    if (!isSquare(matrix)) {
        return;
    }

    int sign = 0;
    unsigned long order = matrix->numRows();
    Matrix<T> cofactors(order, order);

    for (unsigned long i = 0; i < order; i++) {
        for (unsigned long j = 0; j < order; j++) {
            cofactor(matrix, &cofactors, i, j, order);
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            temp->at(j, i) = sign * determinant(&cofactors, order - 1);
        }
    }
}

template<typename T>
bool MatrixManipulator::isSquare(const Matrix<T> *matrix) {
    return (matrix->numCols() == matrix->numRows());
}

template<typename T>
bool MatrixManipulator::sameShape(const Matrix<T> *matrix, const Matrix<T> *other) {
    return matrix->numRows() == other->numRows() && matrix->numCols() == other->numCols();
}

template<typename T>
bool MatrixManipulator::canMultiply(const Matrix<T> *result, const Matrix<T> *matrixA, const Matrix<T> *matrixB) {
    return (result && matrixA && matrixB) &&
            (result->numRows() == matrixA->numRows() && result->numCols() == matrixB->numCols()) &&
            (matrixA->numCols() == matrixB->numRows());
}

//float MatrixManipulator::plus(float &a, const float &b) {
//    return a += b;
//}
//
//float MatrixManipulator::minus(float &a, const float &b) {
//    return a -= b;
//}
//
//float MatrixManipulator::multi(float &a, const float &b) {
//    return a * b;
//}

#endif //MULTIAGENTSLAM_MATRIXMANIPULATOR_H
