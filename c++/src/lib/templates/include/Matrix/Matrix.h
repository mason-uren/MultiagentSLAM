//
// Created by Mason U'Ren on 2019-03-11.
//

#ifndef MULTIAGENTSLAM_MATRIX_H
#define MULTIAGENTSLAM_MATRIX_H

#define EIGEN_USE_MKL_ALL
#define EPSILON 1

#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

template <class T>
class Matrix {
public:
    Matrix(const unsigned long &rows, const unsigned long &cols = 1) :
        data(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(std::max(rows, (u_long) 1), std::max(cols, (u_long) 1))),
        nRows(rows),
        nCols(cols),
        trans(false)
    {}
    Matrix(const std::initializer_list<const std::initializer_list<T>> list) :
        data(
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(
                        list.size() < 2 ? list.begin()->size() : list.size(),
                        list.size() < 2 ? list.size() : list.begin()->size())
            ),
        nRows((u_long) list.size() < 2 ? list.begin()->size() : list.size()),
        nCols((u_long) list.size() < 2 ? list.size() : list.begin()->size()),
        trans(false)
    {
        int i = 0; int j = 0;
        for (auto innerList : list) {
            for (auto val : innerList) {
                data(i, j++) = val;
            }
            i++;
            j = 0;
        }
    }
    Matrix(const Matrix &matrix) :
        data(matrix.data),
        nRows(matrix.nRows),
        nCols(matrix.nCols),
        trans(matrix.trans)
    {}
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
    explicit Matrix(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix) :
            data(matrix),
            nRows((u_long) matrix.rows()),
            nCols((u_long) matrix.cols()),
            trans(false)
    {}

    friend class MatrixManipulator;
    Matrix<T> operator*(const Matrix<T> &matrix) const;
    void operator*=(const float &scalar);
    void operator-=(const Matrix<T> &matrix);
    void operator+=(const Matrix<T> &matrix);

    Eigen::SparseMatrix<T> makeSparse(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &data) const;
    void makeDense(const Eigen::SparseMatrix<T> &matrix);

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data{};
    unsigned long nRows{};
    unsigned long nCols{};
    bool trans{};
};

template <typename T>
bool Matrix<T>::operator==(const Matrix<T> &matrix) const {
    return data.isApprox(matrix.data, 1E-3);
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
    if (data.isZero()) {
        return;
    }
    if (canInvert()) {
        data = data.inverse();
    }
}

template<class T>
bool Matrix<T>::canInvert() {
    if (std::isinf(1 / data.determinant())) {
        std::cerr << "Matrix not invertible. Det: " <<  data.determinant() << std::endl;
        exit(EXIT_FAILURE);
    }
    return true;
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
    return Matrix<T>(Eigen::SparseMatrix<T>(makeSparse(data) * makeSparse(matrix.data)));
}

template<class T>
void Matrix<T>::operator*=(const float &scalar) {
    makeDense(Eigen::SparseMatrix<T>(makeSparse(data) *= scalar));
}

template<class T>
void Matrix<T>::operator-=(const Matrix<T> &matrix) {
    makeDense(Eigen::SparseMatrix<T>(makeSparse(data) -= makeSparse(matrix.data)));
}

template<class T>
void Matrix<T>::operator+=(const Matrix<T> &matrix) {
    makeDense(Eigen::SparseMatrix<T>(makeSparse(data) += makeSparse(matrix.data)));
}

template<class T>
Eigen::SparseMatrix<T> Matrix<T>::makeSparse(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &denseMatrix) const {
    return denseMatrix.sparseView();
}

template<class T>
void Matrix<T>::makeDense(const Eigen::SparseMatrix<T> &matrix) {
    data = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>(matrix);
}

#endif //MULTIAGENTSLAM_MATRIX_H
