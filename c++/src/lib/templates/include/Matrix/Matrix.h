//
// Created by Mason U'Ren on 2019-03-11.
//

#ifndef MULTIAGENTSLAM_MATRIX_H
#define MULTIAGENTSLAM_MATRIX_H

#define EIGEN_USE_MKL_ALL

#include <iostream>
#include <algorithm>
#include <Eigen/Dense>


template <class T>
class Matrix {
public:
    Matrix(const unsigned long &rows, const unsigned long &cols = 1) :
        data(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(std::max(rows, (u_long) 1), std::max(cols, (u_long) 1))),
        nRows(rows),
        nCols(cols),
        trans(false)
    {}
    Matrix(const std::initializer_list<T> list) :
        data(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, list.size())),
        nRows(1),
        nCols((u_long) list.size()),
        trans(false)
    {
        int i = 0;
        for (auto val : list) {
            data(i++) = val;
        }
    }
    Matrix() = default;
    ~Matrix() = default;

    // TODO may need to build copy constructor

    // TODO may ?need to build string parser for 'getType' to elimate garbage characters

    bool operator==(const Matrix &matrix) const;
    void operator=(const Matrix &matrix);
    T &at(const unsigned long &row, const unsigned long &col);
    T &at(const unsigned long &index);
    void transpose();
    void invert();
    bool canInvert();
    unsigned long numRows() const;
    unsigned long numCols() const;
    void resetMatrix();
    void zeroMatrix();

    // For testing purposes only
    void print();

private:
    explicit Matrix(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix) :
            data(matrix),
            nRows((u_long) matrix.rows()),
            nCols((u_long) matrix.cols()),
            trans(false)
    {}

    friend class MatrixManipulator;
    Matrix<T> operator*(const Matrix<T> &matrix) const;
    void operator-=(const Matrix<T> &matrix);
    void operator+=(const Matrix<T> &matrix);

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data{};
    unsigned long nRows{};
    unsigned long nCols{};
    bool trans{};
};

template <typename T>
bool Matrix<T>::operator==(const Matrix<T> &matrix) const {
    return data.isApprox(matrix.data);
}

template<class T>
void Matrix<T>::operator=(const Matrix &matrix) {
    data = matrix.data;
    nRows = matrix.nRows;
    nCols = matrix.nCols;
    trans = matrix.trans;
}

template <typename T>
T &Matrix<T>::at(const unsigned long &row, const unsigned long &col) {
    if (row >= nRows) {
        std::cout << "Matrix : row indice out of bounds... Exiting" << std::endl;
        exit(EXIT_FAILURE);
    } else if (col >= nCols) {
        std::cout << "Matrix : col indice out of bounds... Exiting" << std::endl; ;
        exit(EXIT_FAILURE);
    }
    return data(row, col);
}

template<class T>
T &Matrix<T>::at(const unsigned long &index) {
    if (index >= (nRows * nCols)) {
        std::cout << "Vector : index out of bounds... Exiting" << std::endl;
        exit(EXIT_FAILURE);
    }
    return data(index);
}

template <typename T>
void Matrix<T>::transpose() {
    trans = !trans;
    std::swap(nRows, nCols);
    data.transposeInPlace();
}

template<class T>
void Matrix<T>::invert() {
    if (canInvert()) {
        data = data.inverse();
    }
}

template<class T>
bool Matrix<T>::canInvert() {
    return !std::isinf(1 / data.determinant()) ;
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
    std::cout << data << std::endl;
}

template<class T>
void Matrix<T>::resetMatrix() {
    if (trans) {
        trans = !trans;
        std::swap(nRows, nCols);
    }
    data = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(nRows, nCols);
}

template <typename T>
void Matrix<T>::zeroMatrix() {
    data = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(nRows, nCols);
}

template<class T>
Matrix<T> Matrix<T>::operator*(const Matrix<T> &matrix) const {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> result = data * matrix.data;
    return Matrix<T>(result);
}

template<class T>
void Matrix<T>::operator-=(const Matrix<T> &matrix) {
    data -= matrix.data;
}

template<class T>
void Matrix<T>::operator+=(const Matrix<T> &matrix) {
    data += matrix.data;
}

#endif //MULTIAGENTSLAM_MATRIX_H
