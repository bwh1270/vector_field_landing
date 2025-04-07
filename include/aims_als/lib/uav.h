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
 * @file uav.h 
 * This class is for easily maniplating the states of UAV by subscribing 
 * /mavros/local_position/odom and /mavros/imu/data topics
 * 
 * @author Woohyun Byun - imbwh@cau.ac.kr
 * 
 * @date 2024.10.28
 */ 

#ifndef __UAV_H__
#define __UAV_H__

#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <eigen3/Eigen/Dense>

#include "aims_als/lib/util.h"
#include "aims_als/lib/math.h"
#include "aims_als/lib/filter.h"

using namespace aims_fly;


/**
 * @brief Frames
 *     - position, velocity and acceleration of origin of UAV frame in world frame
 *     - orientation of UAV w.r.t. world frame (quaternion, Euler angles, Rotation matrix, head (i.e. azimuth))
 */

struct timestamp_t
{
    double odom;
    double imu;
};

class UAV
{
    public:
        UAV();
        ~UAV() = default;

        // Helpful variables
        timestamp_t t_;
        Eigen::Vector3d eW_z_;
        Eigen::Vector2d eW_x_e12_;

        // States
        Eigen::Vector3d p_, v_, a_;
        Eigen::Vector4d q_;
        Eigen::Vector3d E_;
        Eigen::Matrix3d R_;

        // Aims states
        double head_;
        Eigen::Vector3d eB_z_;

        // Params
        bool b_using_imu_atti_;  // If subsribing imu data, then orientation is computed in Imu Callback
        double lpf_acc_alpha_, lpf_vel_alpha_;


        // Functions
        void measureAttitudeFromImu(bool b_imu);
        
        void updatePX4OdomState(const double t, const Eigen::Vector3d &p, const Eigen::Vector4d &q, const Eigen::Vector3d &v);
        void updateOdomState(const double t, const Eigen::Vector3d &p, const Eigen::Vector4d &q, const Eigen::Vector3d &v);

        void setLpfAccAlpha(const double alpha);
        void setLpfVelAlpha(const double alpha);
        void updateImuState(const double t, const Eigen::Vector3d &a, const Eigen::Vector4d &q);

        // For debugs
        void stateMonitoring();

    private:
        // double computeHead(const Eigen::Vector4d &qW_B, const Eigen::Vector3d &eB_z);
        double computeHead(const Eigen::Vector3d &eB_x);

};

UAV::UAV()
{
    // Init helpful variables
    t_.odom = 0.;
    t_.imu  = 0.;
    eW_z_ << 0., 0., 1.;
    eW_x_e12_ << 1., 0.;
    
    // Init the states
    p_ << 0., 0., 0.;
    v_ << 0., 0., 0.;
    a_ << 0., 0., 0.;
    q_ << 1., 0., 0., 0.;
    E_ << 0., 0., 0.;
    R_ << 1., 0., 0.,
          0., 1., 0.,
          0., 0., 1.;
        
    // Init the Aims states
    head_ = 0.;
    eB_z_ << 0., 0., 1.;

    // Init the parameters
    b_using_imu_atti_ = false;
    lpf_acc_alpha_ = 1.;
    lpf_vel_alpha_ = 1.;
}

void UAV::measureAttitudeFromImu(bool b_imu)
{
    b_using_imu_atti_ = b_imu;
}

void UAV::updatePX4OdomState(const double t, const Eigen::Vector3d &p, const Eigen::Vector4d &q, const Eigen::Vector3d &v)
{
    t_.odom = t;
    p_ = p;
    
    const Eigen::Vector3d v_prev = v_;
    
    const Eigen::Matrix3d R = q2R(q);
    v_ = R*v;  // "/mavros/local_position/odom" topic is expressed in body frame 
    
    v_ = lpfv3(v, lpf_vel_alpha_, v_prev);  

    if (!b_using_imu_atti_) {
        q_ = q;
        E_ = q2E(q);
        R_ = q2R(q);
        eB_z_ = R_ * Eigen::Vector3d(0.,0.,1.);
        head_ = computeHead(R_ * Eigen::Vector3d(1.,0.,0.));
        // head_ = E_(2); // only for uav setpoint 
    }
}

void UAV::updateOdomState(const double t, const Eigen::Vector3d &p, const Eigen::Vector4d &q, const Eigen::Vector3d &v)
{
    t_.odom = t;
    p_ = p;

    // this velocity is expressed in world frame 
    const Eigen::Vector3d v_prev = v_;
    v_ = lpfv3(v, lpf_vel_alpha_, v_prev);  
    
    if (!b_using_imu_atti_) {
        q_ = q;
        E_ = q2E(q);
        R_ = q2R(q);
        eB_z_ = R_ * Eigen::Vector3d(0.,0.,1.);
        head_ = computeHead(R_ * Eigen::Vector3d(1.,0.,0.));
    }
}

void UAV::setLpfAccAlpha(const double alpha)
{
    lpf_acc_alpha_ = alpha;
}

void UAV::setLpfVelAlpha(const double alpha)
{
    lpf_vel_alpha_ = alpha;
}

void UAV::updateImuState(const double t, const Eigen::Vector3d &a, const Eigen::Vector4d &q)
{
    t_.imu = t;
    const Eigen::Vector3d a_w = rotVecWithQuat(q, a);
    const Eigen::Vector3d a_prev = a_;
    a_ = lpfv3(a_w, lpf_acc_alpha_, a_prev);
    
    if (b_using_imu_atti_) {
        q_ = q;
        E_ = q2E(q);
        R_ = q2R(q);
        eB_z_ = R_ * eW_z_;
        head_ = computeHead(R_ * Eigen::Vector3d(1.,0.,0.));
    }
}   


/**
 * @brief Computation Functions
 */

double UAV::computeHead(const Eigen::Vector3d &eB_x)
{
    const Eigen::Vector2d eB_x_e12(eB_x(0), eB_x(1));
    const double head = acos(eB_x_e12.dot(eW_x_e12_) / eB_x_e12.norm());
    
    // 1&2 quadrant -> head>0
    // 3&4 quadrant -> head<0
    if (eB_x(1) >= 0) {
        return head;
    } else {
        return -head;
    }
}


/**
 * @brief Debugging Functions
 */
void UAV::stateMonitoring()
{
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "Times:" << std::endl;
    std::cout << "Imu Callback:  " << t_.imu << std::endl;
    std::cout << "Odom Callback: " << t_.odom << std::endl;
    
    std::cout << "\nStates:" << std::endl;
    std::cout << "Position:     [x,y,z] = [" << p_.transpose() << "]" << std::endl;
    std::cout << "Velocity:     [x,y,z] = [" << v_.transpose() << "]" << std::endl;
    std::cout << "Acceleration: [x,y,z] = [" << a_.transpose() << "]" << std::endl;
    std::cout << "Quat:  [w,x,y,z] = [" << q_.transpose() << "]" << std::endl;
    std::cout << "Euler: [r,p,y] = [" << (180./PI * E_).transpose() << "]" << std::endl;

    std::cout << "Head: " << head_ << std::endl;
    std::cout << "eB_z: [" << eB_z_.transpose() << "]" << std::endl;
}

#endif