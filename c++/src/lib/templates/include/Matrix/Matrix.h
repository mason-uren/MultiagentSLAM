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
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>
#include <Eigen/SparseLU>
#include <Eigen/OrderingMethods>

template <class T>
class Matrix {
public:
    Matrix(const unsigned long &rows, const unsigned long &cols = 1) :
            data(Eigen::SparseMatrix<T, Eigen::ColMajor, long> (
                    std::max(rows, (u_long) 1), std::max(cols, (u_long) 1))),
            nRows(rows),
            nCols(cols),
            trans(false)
    {}
    Matrix(const std::initializer_list<const std::initializer_list<T>> list) :
            data(
                    Eigen::SparseMatrix<T, Eigen::ColMajor, long>(
                            list.size() < 2 ? list.begin()->size() : list.size(),
                            list.size() < 2 ? list.size() : list.begin()->size())
            ),
            nRows((u_long) list.size() < 2 ? list.begin()->size() : list.size()),
            nCols((u_long) list.size() < 2 ? list.size() : list.begin()->size()),
            trans(false)
    {
        std::vector<triplet> tripletList(nRows * nCols);
        int i = 0; int j = 0;
        for (auto innerList : list) {
            for (auto val : innerList) {
                tripletList[(i * nCols) + j] = triplet(i, j, val);
                j++;
            }
            i++;
            j = 0;
        }
        data.setFromTriplets(tripletList.begin(), tripletList.end());
    }
    Matrix(const Matrix &matrix) :
        data(matrix.data),
        nRows(matrix.nRows),
        nCols(matrix.nCols),
        trans(matrix.trans)
    {}
    Matrix() = default;
    ~Matrix() = default;

    bool operator==(const Matrix &matrix) const;
    void operator=(const Matrix &matrix);
    T &at(const unsigned long &row, const unsigned long &col);
    T &at(const unsigned long &index);
    void identity();
    void transpose();
    void invert();
    bool canInvert();
    unsigned long numRows() const;
    unsigned long numCols() const;
    void resetMatrix();
    void zeroMatrix();

    // For testing purposes only
    void print();

    Matrix<T> operator*(const Matrix<T> &matrix) const;
    Matrix<T> operator-(const Matrix<T> &matrix);
    Matrix<T> operator+(const Matrix<T> &matrix);
    void operator*=(const float &scalar);
    void operator-=(const Matrix<T> &matrix);
    void operator+=(const Matrix<T> &matrix);

private:
    explicit Matrix(const Eigen::SparseMatrix<T, Eigen::ColMajor, long> &matrix) :
            data(matrix),
            nRows((u_long) matrix.rows()),
            nCols((u_long) matrix.cols()),
            trans(false)
    {}

    typedef Eigen::Triplet<T> triplet;

    Eigen::SparseMatrix<T> data{};
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
    return data.coeffRef(row, col);
}

template<class T>
T &Matrix<T>::at(const unsigned long &index) {
    if (index >= (nRows * nCols)) {
        std::cout << "Vector : index out of bounds... Exiting" << std::endl;
        exit(EXIT_FAILURE);
    }
    return data.coeffRef(index, 0);
}

template<class T>
void Matrix<T>::identity() {
    data.setIdentity();
}

template <typename T>
void Matrix<T>::transpose() {
    trans = !trans;
    std::swap(nRows, nCols);
    data = Eigen::SparseMatrix<T>(data.transpose());
}

template<class T>
void Matrix<T>::invert() {
    if (!data.nonZeros()) {
        return;
    }
//    if (canInvert()) {
        Eigen::SparseQR<Eigen::SparseMatrix<T, Eigen::ColMajor, long>, Eigen::COLAMDOrdering<long>> solver;
        solver.compute(data);
        Eigen::SparseMatrix<T, Eigen::ColMajor, long> I(nRows, nCols);
        I.setIdentity();
        data = solver.solve(I);
//    }
}

template<class T>
bool Matrix<T>::canInvert() {
    Eigen::SparseLU<Eigen::SparseMatrix<T, Eigen::ColMajor, long>, Eigen::COLAMDOrdering<long>> solver;
    solver.analyzePattern(data);
    solver.factorize(data);
    float val = solver.logAbsDeterminant();

    if (std::isinf(val)) {
        std::cerr << "Matrix not invertible. Det: " <<  val << std::endl;
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
    std::cout << Eigen::Matrix<T, -1, -1>(data) << std::endl;
}

template<class T>
void Matrix<T>::resetMatrix() {
    if (trans) {
        trans = !trans;
        std::swap(nRows, nCols);
    }
    data = Eigen::SparseMatrix<T, Eigen::ColMajor, long>(nRows, nCols);
}

template <typename T>
void Matrix<T>::zeroMatrix() {
    data.setZero();
}

template<class T>
Matrix<T> Matrix<T>::operator*(const Matrix<T> &matrix) const {
    if (nCols != matrix.nRows) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    return Matrix<T>((data * matrix.data).pruned(1E-3));
}

template<class T>
Matrix<T> Matrix<T>::operator-(const Matrix<T> &matrix) {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    return Matrix<T>(data - matrix.data);
}

template<class T>
Matrix<T> Matrix<T>::operator+(const Matrix<T> &matrix) {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    return Matrix<T>(data + matrix.data);
}



template<class T>
void Matrix<T>::operator*=(const float &scalar) {
    data *= scalar;
}

template<class T>
void Matrix<T>::operator-=(const Matrix<T> &matrix) {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    data -= matrix.data;
}

template<class T>
void Matrix<T>::operator+=(const Matrix<T> &matrix) {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    data += matrix.data;
}

#endif //MULTIAGENTSLAM_MATRIX_H
