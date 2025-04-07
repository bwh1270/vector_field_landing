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
 * @file math.h 
 * This file is used to compute the orientation.
 * 
 * @author Woohyun Byun - imbwh@cau.ac.kr
 * 
 * @date 2024.10.29
 */ 

#ifndef __MATH_H__
#define __MATH_H__

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace aims_fly
{

constexpr double N_EPS = 1e-10;  // numerical epsilon
constexpr double PI    = 3.14159265358979323846;
constexpr double h_PI  = 1.57079632679489661923;
constexpr double t_PI  = PI*2.;

inline double rad2deg(const double rad)
{
    return rad * 180. / PI;
}

inline double deg2rad(const double deg)
{
    return deg * PI / 180.;
}

inline double wrap2PI(const double x) 
{
    double y = fmod(x+PI, t_PI);
    if (y < 0) {
        y += t_PI;
    }
    return y - PI;
}

template <typename T>
inline int sgn(const T x)
{
    if (x >= 0) {
        return 1;
    } else {
        return -1;
    }
}

/**
 * @brief Quaternion
 * Elements of quaternion are following the order: q = [w,x,y,z]
 */

inline Eigen::Vector4d axisAngle2Quat(const Eigen::Vector3d &l, const double angle)
{
    Eigen::Vector4d q;
    const double angle_h = angle * 0.5; 
    q << cos(angle_h), l(0)*sin(angle_h), l(1)*sin(angle_h), l(2)*sin(angle_h);
    return q;
}

inline Eigen::Matrix3d q2R(const Eigen::Vector4d &q)
{
    Eigen::Matrix3d R;

    Eigen::Vector4d qq;
    for (int i=0; i<4; i++) {
        qq(i) = q(i)*q(i);
    }
    R(0,0) = qq(0)+qq(1)-qq(2)-qq(3);
    R(0,1) = 2.*(q(1)*q(2)-q(0)*q(3));
    R(0,2) = 2.*(q(1)*q(3)+q(0)*q(2));

    R(1,0) = 2.*(q(1)*q(2)+q(0)*q(3));
    R(1,1) = qq(0)+qq(2)-qq(1)-qq(3);
    R(1,2) = 2.*(q(2)*q(3)-q(0)*q(1));

    R(2,0) = 2.*(q(1)*q(3)-q(0)*q(2));
    R(2,1) = 2.*(q(2)*q(3)+q(0)*q(1));
    R(2,2) = qq(0)+qq(3)-qq(1)-qq(2);

    return R;
}

inline Eigen::Vector3d q2E(const Eigen::Vector4d &q)
{
    Eigen::Vector3d E;
    Eigen::Vector4d qq;
    for (int i=0; i<4; i++) {
        qq(i) = q(i)*q(i);
    }

    E(0) = atan2(2.*(q(2)*q(3)+q(0)*q(1)), (qq(0)+qq(3)-qq(1)-qq(2)));
    E(1) = -asin(2.*(q(1)*q(3)-q(0)*q(2)));
    E(2) = atan2(2.*(q(1)*q(2)+q(0)*q(3)), (qq(0)+qq(1)-qq(2)-qq(3)));
    
    return E; 
}

inline Eigen::Vector4d vec2QuatVec(const Eigen::Vector3d &u)
{
    Eigen::Vector4d u_hat;
    u_hat << 0., u(0), u(1), u(2);
    return u_hat;
}

inline Eigen::Vector3d quatVec2vec(const Eigen::Vector4d &u_hat)
{
    Eigen::Vector3d u;
    u << u_hat(1), u_hat(2), u_hat(3);
    return u;
}

inline Eigen::Vector4d setQuatConjugate(const Eigen::Vector4d &q)
{
    Eigen::Vector4d q_star;
    q_star << q(0), -q(1), -q(2), -q(3);
    return q_star;
} 

inline Eigen::Matrix4d setQuatMatrix(const Eigen::Vector4d &q)
{
    Eigen::Matrix4d Q;
    Q << q(0), -q(1), -q(2), -q(3),
         q(1),  q(0), -q(3),  q(2),
         q(2),  q(3),  q(0), -q(1),
         q(3), -q(2),  q(1),  q(0);
    return Q;
}

inline Eigen::Vector4d quatProd(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2)
{
    const Eigen::Matrix4d Q = setQuatMatrix(q1);
    return Q*q2;
}

inline Eigen::Vector3d rotVecWithQuat(const Eigen::Vector4d &q, const Eigen::Vector3d ub)
{
    Eigen::Vector4d q_normalized = q / q.norm();
    Eigen::Vector4d u_hat = vec2QuatVec(ub);
    Eigen::Vector4d q_tmp = quatProd(q_normalized, u_hat);
    Eigen::Vector4d q_conj = setQuatConjugate(q_normalized);
    Eigen::Vector3d ua = quatVec2vec(quatProd(q_tmp, q_conj));
    return ua;
}


/**
 * @brief Euler Angles
 * [roll pitch yaw]
 */

inline Eigen::Vector4d E2q(const Eigen::Vector3d &E)
{
    Eigen::Vector4d q;

    q(0) = cos(E(0)/2.)*cos(E(1)/2.)*cos(E(2)/2.) + sin(E(0)/2.)*sin(E(1)/2.)*sin(E(2)/2.);
    q(1) = sin(E(0)/2.)*cos(E(1)/2.)*cos(E(2)/2.) - cos(E(0)/2.)*sin(E(1)/2.)*sin(E(2)/2.);
    q(2) = cos(E(0)/2.)*sin(E(1)/2.)*cos(E(2)/2.) + sin(E(0)/2.)*cos(E(1)/2.)*sin(E(2)/2.);
    q(3) = cos(E(0)/2.)*cos(E(1)/2.)*sin(E(2)/2.) - sin(E(0)/2.)*sin(E(1)/2.)*cos(E(2)/2.);

    return q;
}

inline Eigen::Matrix3d E2R(const Eigen::Vector3d &E)
{
    // Rotaion: ZYX
    Eigen::Matrix3d R;

    R(0,0) = cos(E(2))*cos(E(1));
    R(0,1) = -sin(E(2))*cos(E(0))+cos(E(2))*sin(E(1))*sin(E(0));
    R(0,2) = sin(E(2))*sin(E(0))+cos(E(2))*sin(E(1))*cos(E(0));

    R(1,0) = sin(E(2))*cos(E(1));
    R(1,1) = cos(E(2))*cos(E(0))+sin(E(2))*sin(E(1))*sin(E(0));
    R(1,2) = -cos(E(2))*sin(E(0))+sin(E(2))*sin(E(1))*cos(E(0));

    R(2,0) = -sin(E(1)); 
    R(2,1) = cos(E(1))*sin(E(0));
    R(2,2) = cos(E(1))*cos(E(0));

    return R;
}


/**
 * @brief Rotation Matrix (DCM)
 * [b1 b2 b3]
 */

inline Eigen::Vector4d R2q(const Eigen::Matrix3d &R)
{
    Eigen::Vector4d q;

    const double trace = R.trace();
    if (trace > 0) {
        const double s = 0.5 / sqrt(trace + 1.);
        q(0) = 0.25 / s;
        q(1) = (R(2,1) - R(1,2)) * s;
        q(2) = (R(0,2) - R(2,0)) * s;
        q(3) = (R(1,0) - R(0,1)) * s;
    } else {
        if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
            const double s = 2. * sqrt(1. + R(0,0) - R(1,1) - R(2,2));
            q(0) = (R(2,1) - R(1,2)) / s;
            q(1) = 0.25 * s;
            q(2) = (R(0,1) + R(1,0)) / s;
            q(3) = (R(0,2) + R(2,0)) / s;
        } else if (R(1,1) > R(2,2)) {
            const double s = 2. * sqrt(1. + R(1,1) - R(0,0) - R(2,2));
            q(0) = (R(0,2) - R(2,0)) / s;
            q(1) = (R(0,1) + R(1,0)) / s;
            q(2) = 0.25 * s;
            q(3) = (R(1,2) + R(2,1)) / s;
        } else {
            const double s = 2. * sqrt(1. + R(2,2) - R(0,0) - R(1,1));
            q(0) = (R(1,0) - R(0,1)) / s;
            q(1) = (R(0,2) + R(2,0)) / s;
            q(2) = (R(1,2) + R(2,1)) / s;
            q(3) = 0.25 * s;
        }
    }
    return q;
}

inline Eigen::Vector3d R2E(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d E;

    E(0) = atan2(R(2,1), R(2,2));
    E(1) = -asin(R(2,0));
    E(2) = atan2(R(1,0), R(0,0));

    return E;
}

inline Eigen::Vector3d rotVecWithR(const Eigen::Matrix3d &R, const Eigen::Vector3d &u)
{
    return R*u;
}

inline Eigen::Matrix4d setMatrixT(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Zero(4,4);
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    T(3,3) = 1.;
    return T;
}

inline Eigen::Vector3d rotVecWithRt(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const Eigen::Vector3d &ub)
{
    Eigen::Vector3d ua;
    
    Eigen::Vector4d ua_hat, ub_hat;
    ub_hat << ub(0), ub(1), ub(2), 1.;
    const Eigen::Matrix4d T = setMatrixT(R, t);
    ua_hat = T*ub_hat;
    ua << ua_hat(0), ua_hat(1), ua_hat(2);

    return ua;
}

inline Eigen::Vector3d rotVecWithT(const Eigen::Matrix4d &T, const Eigen::Vector3d &ub)
{
    Eigen::Vector3d ua;
    
    Eigen::Vector4d ua_hat, ub_hat;
    ub_hat << ub(0), ub(1), ub(2), 1.;

    ua_hat = T*ub_hat;
    ua << ua_hat(0), ua_hat(1), ua_hat(2);

    return ua;
}


/**
 * Operations b/w Two Vectors 
 */

inline double computeAngleBwTwoVec(const Eigen::Vector3d u, const Eigen::Vector3d v)
{
    return acos(u.dot(v) / (u.norm() * v.norm())); 
}

inline Eigen::Vector3d computeRotAxis(const Eigen::Vector3d u, const Eigen::Vector3d v)
{
    return u.cross(v) / (u.cross(v)).norm();
}

/**
 * Mapping Functions
 */

inline double sigmoid(const double x)
{
    return 1.0 / (1.0 + std::exp(-x));
}

inline double tanh_derivative(double x)
{
    double tanh_x = std::tanh(x);
    return 1.0 - tanh_x * tanh_x;
}


};

#endif