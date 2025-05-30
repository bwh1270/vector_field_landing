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
 * @brief Position Control 
 * @author Woohyun Byun - imbwh@cau.ac.kr
 * @date 2024.10.28
 */
#ifndef __POSITION_CONTROL_OLD__
#define __POSITION_CONTROL_OLD__

#include <iostream>
#include <cassert>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

// Input types
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/State.h"
#include "std_msgs/Bool.h"

// Output types
#include "mavros_msgs/AttitudeTarget.h"

#include "vector_field_landing/lib/util.h"
#include "vector_field_landing/lib/math.h"
#include "vector_field_landing/lib/uav.h"

using namespace aims_fly;


struct pidGain_t
{
    Eigen::Matrix3d P;
    Eigen::Matrix3d I;
    Eigen::Matrix3d D;
};

class PositionControl
{
    public:
        PositionControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~PositionControl() = default;

        void run() { ros::spin(); };

    
    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_odom_uav_;
        ros::Subscriber sub_imu_uav_;
        ros::Subscriber sub_state_uav_;
        ros::Subscriber sub_hover_thrust_;
        ros::Subscriber sub_odom_esti_;
        ros::Subscriber sub_bool_land_;
        ros::Publisher  pub_attiTarget_;

        // Constants
        const Eigen::Vector3d eW_z_;
        const Eigen::Vector3d g_;

        // Control State
        UAV uav_;
        UAV uav_sp_;
        std::string mode_;
        Eigen::Vector3d asp_;
        Eigen::Vector4d qW_D_;

        bool land_;

        // Control Action
        double _vel_xy_max, _vel_z_max;                     // max horizontal/vertical velocity limit for safety

        Eigen::Vector3d int_vel_;                           // integral error of Position controller
        double _int_vel_xy_max, _int_vel_z_max;             // limitations for growing integral of error excessively

        double _thr_min, _thr_max;                          // min/max normalized thrust limit for safety
        double _tilt_max;                                   // max tilt limit for safety

        // Control Gain
        pidGain_t P_, V_;

        // Parameter
        bool _with_pos;
        bool _for_gvf;

        double _dt_c;                                       // to avoid numerical issues (dt = 0)
        
        double _lpf_acc_alpha;                              // LPF parameter proportional to cutoff frequency for acceleration from IMU
        double _lpf_ref_alpha;                              // LPF parameter for filtering feedforward path (a little higher than ref FFT result)
        double _lpf_vel_alpha;

        double _thr_hover_init;                             // normalized hover thrust initial value
        double thr_hover_esti_;                             // normalized hover thrust estimate

        // Callback Function
        void uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
        void uavImuCb(const sensor_msgs::Imu::ConstPtr &msg);
        void uavStateCb(const mavros_msgs::State::ConstPtr &msg);
        void hoverThrustCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void gvEstiOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
        void landCb(const std_msgs::Bool::ConstPtr &msg);

        // Control
        void posCtrl(UAV &uav_sp, const UAV &uav);
        void velCtrl(UAV &uav_sp, const UAV &uav, const double dt);

        Eigen::Vector3d limitTilt(const Eigen::Vector3d &eD_z);
        Eigen::Vector4d computeDesiredAttitude(const double head_sp, const double head, const Eigen::Vector3d &eD_z);
        double computeCollectiveThrust(const Eigen::Vector3d &asp_k, const Eigen::Vector3d &eB_z);
        
        void accCtrl();

        void publish(const Eigen::Vector4d &qW_D, const double Tc);
};


#endif