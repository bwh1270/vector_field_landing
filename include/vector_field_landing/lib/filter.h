/*********************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Woohyun Byun.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ********************************************************************************/
/**
 * @file filter.h 
 * This file is used to adjust the filters:
 * - low-pass filter.
 * 
 * @author Woohyun Byun - imbwh@cau.ac.kr
 * 
 * @date 2024.10.28
 */ 

#ifndef __FILTER_H__
#define __FILTER_H__

#include <iostream>
#include <cmath>
#include <vector>
#include <deque>
#include <eigen3/Eigen/Dense>

namespace aims_fly
{

/**
 * Low-pass Filter
 * 
 * @example
 * - float result_f = lpf(0.5f, 0.1f, 0.4f); 
 * - double result_d = lpf(0.5, 0.1, 0.4);  
 * 
 * @example
 * - Eigen::Vector3f v_f, v, prev_v_f; 
 * float alpha = 0.1f;
 * LPFv3(v, alpha, prev_v_f, v_f);  // Works with Vector3f
 * 
 * @example
 * Eigen::Vector3d v_d, prev_v_d, v_f_d;
 * double alpha_d = 0.1;
 * LPFv3(v_d, alpha_d, prev_v_d, v_f_d);  // Works with Vector3d
 */

template <typename T>
inline T lpf(const T &x, const T &alpha, const T &prev_x_f)
{
    T x_f = alpha*x + (1-alpha)*prev_x_f;
    return x_f;
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> lpfv3(const Eigen::Matrix<T, 3, 1> &v, const T &alpha, const Eigen::Matrix<T, 3, 1> &prev_v_f)
{
    Eigen::Matrix<T, 3, 1> v_f = alpha * v + (1 - alpha) * prev_v_f;
    return v_f;
}

/**
 * Savitzky-Golay Filter
 */

std::vector<double> computeSavitzkyGolayCoefficients(int window_size, int poly_order, int deriv_order) {
    if (window_size % 2 == 0 || window_size <= poly_order) {
        throw invalid_argument("Window size must be odd and greater than poly order.");
    }

    int half = window_size / 2;
    Eigen::MatrixXd A(window_size, poly_order + 1);

    // 디자인 행렬 A 생성
    for (int i = -half; i <= half; ++i) {
        for (int j = 0; j <= poly_order; ++j) {
            A(i + half, j) = pow(i, j);
        }
    }

    // (A^T * A)^-1 * A^T
    Eigen::MatrixXd ATA_inv = (A.transpose() * A).inverse();
    Eigen::MatrixXd pseudo_inv = ATA_inv * A.transpose();

    // 도함수 차수에 해당하는 행 선택
    Eigen::VectorXd coeffs = pseudo_inv.row(deriv_order);

    return std::vector<double>(coeffs.data(), coeffs.data() + coeffs.size());
}

inline double applySavitzkyGolay(const std::deque<double> &x, const std::vector<double> &coeffs, double dt, int deriv_order)
{
    size_t window_size = coeffs.size();

    if (x.size() != window_size) {
        throw std::invalid_argument("Deque size must match coefficient size.");
    }

    double val = 0.0;
    for (size_t i = 0; i < window_size; ++i) {
        val += coeffs[i] * x[i];
    }

    return val / std::pow(dt, deriv_order);
}

};

#endif