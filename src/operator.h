#ifndef OPERATOR_H
#define OPERATOR_H

#include <iostream>
#include <vector>
#include <aris.hpp>
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
using namespace aris::dynamic;
using namespace aris::plan;

template<typename T>
class Matrix {
public:
    Matrix(size_t rows, size_t cols);

    // 获取矩阵的行数
    size_t rows() const;

    // 获取矩阵的列数
    size_t cols() const;

    // 访问矩阵元素
    T& operator()(size_t row, size_t col); // Non-const version
    const T& operator()(size_t row, size_t col) const; // Const version

    // 输出矩阵内容
    void print() const;

private:
    size_t rows_;
    size_t cols_;
    std::vector<std::vector<T>> data_;
};

// 实现构造函数
template<typename T>
Matrix<T>::Matrix(size_t rows, size_t cols) : rows_(rows), cols_(cols), data_(rows, std::vector<T>(cols, 0)) {}

// 实现获取行数函数
template<typename T>
size_t Matrix<T>::rows() const {
    return rows_;
}

// 实现获取列数函数
template<typename T>
size_t Matrix<T>::cols() const {
    return cols_;
}

// 实现访问矩阵元素函数
template<typename T>
T& Matrix<T>::operator()(size_t row, size_t col) {
    return data_[row][col];
}

// 实现输出矩阵内容函数
template<typename T>
void Matrix<T>::print() const {
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            std::cout << data_[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

// 添加其他操作
void otherOperation();

/// <summary>
/// isQuotientOdd
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool isQuotientOdd(int a, int b);
void splitMatrix28(double a[28], double b[16], double c[12]);
void show(int8_t m, int8_t n, double* a);

#endif // OPERATOR_H