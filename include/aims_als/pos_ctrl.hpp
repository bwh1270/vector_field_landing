/*********************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Woohyun Byun.
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
 * @date 2025.03.22
 */

#ifndef __POSITION_CONTROL__
#define __POSITION_CONTROL__

#include <iostream>
#include <fstream>
#include <ctime>
#include <cassert>
#include <chrono>
#include <memory>
#include <string>
#include <deque>
#include <limits>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <random>

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include "std_msgs/Int32.h"

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/AttitudeTarget.h"

#include "aims_als/lib/util.h"
#include "aims_als/lib/math.h"
#include "aims_als/lib/uav.h"

using namespace aims_fly;


class PositionControl
{
    public:
        PositionControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~PositionControl() {
            if (flag_.log) {
                if (log_.is_open()) {
                    log_.close();
                    ROS_INFO("Logging file is closed");
                }
            }
        }

        void run() { ros::spin(); };

    
    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber sub_pose_uav_;
        ros::Subscriber sub_twist_uav_;
        ros::Subscriber sub_imu_uav_;
        ros::Subscriber sub_fm_uav_;
        ros::Subscriber sub_hover_thrust_;

        ros::Subscriber sub_odom_gv_;
        ros::Subscriber sub_fsm_state_;

        ros::ServiceClient set_mode_client_;
        
        ros::Publisher pub_attitarget_;

        ros::Timer monitor_timer_;

        std::ofstream log_;

        // Constants
        const Eigen::Vector3d w3_;
        const Eigen::Vector3d g_;
        double _monitor_rate;
        double _thr_h_const;
        const size_t MAX_SIZE;

        // Flight Mode
        struct fm_t {  
            std::string position;
            std::string hold;
            std::string offboard;
        };

        struct uav_t {
            double t;
            int fsm;
            double fsm_time;
            std::string fm;
            Eigen::Vector3d p;
            Eigen::Vector3d v;
            Eigen::Vector3d v_prev;
            Eigen::Vector3d a;
            Eigen::Matrix3d R;
            Eigen::Vector3d b3;
            double head;
            double v_lpf_freq;
            double a_lpf_freq;
            double v_lpf_alpha;  // lpf for uav velocity 
            double a_lpf_alpha;  // lpf for uav acceleration
            double thr_h;
        };

        struct uav_pose_t {
            double t;
            Eigen::Vector3d p;
            Eigen::Matrix3d R;
            double head;
        };
        std::deque<PositionControl::uav_pose_t> uav_pose_dq_;

        struct uav_twist_t {
            double t;
            Eigen::Vector3d v;
        };
        std::deque<PositionControl::uav_twist_t> uav_twist_dq_;

        struct gv_t {
            double t;
            Eigen::Vector3d p;
            Eigen::Vector3d v;
            double head;
            Eigen::Matrix2d R;
            double v_lpf_freq;  // lpf for gv velocity 
            double h_lpf_freq;  // lpf for gv head
            double v_lpf_alpha;
            double h_lpf_alpha;
        };

        struct rel_t {
            Eigen::Vector3d p;
            Eigen::Vector3d v;
            Eigen::Vector2d eta;
            Eigen::Vector2d eta_dot;
        };

        struct vf_t {
            double phi_des;
            double phi_delta; // boundary = phi_des +- phi_delta
            double r_max;
            double k1;
            double k2;
            double margin;
            double c1;
            double c2;
            double n1;  
            double n2;
            double gamma;
            Eigen::Vector2d h;
            Eigen::Vector3d a_law;
            double a_cmd_alpha;
            bool moving;
        };

        struct cmd_t {
            Eigen::Vector3d standoff;
            Eigen::Vector3d v;
            Eigen::Vector3d int_v;
            Eigen::Vector3d a;
            Eigen::Vector4d q;
            double thr_c;  // collective thrust
            double thr_l;  
        };

        struct dob_t {
            double q_lpf_freq;      // lpf for q-filter of DOB
            double tau_uav;
            double Q_a0;
            double Q_b0;
            double PnQ_b0;
            double PnQ_b1;
            double PnQ_a0;
            Eigen::Vector3d w1;
            Eigen::Vector3d w2;
            Eigen::Vector3d w;
            Eigen::Vector3d u_c;
            Eigen::Vector3d u_dob;
            Eigen::Vector3d u_dob_prev;
        };

        struct sat_t { // [min,max]
            double dt;
            double vel_xy;
            double vel_z;
            double int_vxy;
            double int_vz;
            Eigen::Vector2d thr;
            double tilt;
            double T;
            double crt_alt;
            double acc_ratio;
            double t;
            double t_end;
        };

        struct pid_gain_t {
            Eigen::Matrix3d P;
            Eigen::Matrix3d I;
            Eigen::Matrix3d D;
        };
        
        struct flag_t {
            bool debug;
            bool ff;
            bool dob;
            bool landed;
            bool sitl;
            bool log;
        };

        fm_t  fm_;
        uav_t uav_;
        gv_t  gv_;
        rel_t rel_;
        vf_t  vf_;
        cmd_t cmd_;
        dob_t dob_;
        sat_t sat_;
        pid_gain_t P_, V_;
        flag_t flag_;

        void setFlightMode();
        void initCmd();
        void initSys();
        void initDOB();

        void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void uavTwistCb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void uavImuCb(const sensor_msgs::Imu::ConstPtr &msg);
        void uavFmCb(const mavros_msgs::State::ConstPtr &msg);
        void hoverThrustCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void gvEstiCb(const nav_msgs::Odometry::ConstPtr &msg);
        void fsmCb(const std_msgs::Int32::ConstPtr &msg);
        void monitor(const ros::TimerEvent &event);
        void changeFlightMode(const std::string &mode);

        /** @brief PD with Filtered Feedforward */
        void posCtrl(const Eigen::Vector3d &p_des, const Eigen::Vector3d &v_des, const Eigen::Vector3d &p, const Eigen::Vector3d &v);
        void velCtrl(const Eigen::Vector3d &v_des, const Eigen::Vector3d &v, const Eigen::Vector3d &a, const double dt);
        void accCtrl();
        void acc2thrAtti(const Eigen::Vector3d &a_des, const Eigen::Vector3d &b3);
        void estimateDst();

        double computeHead(const Eigen::Vector3d &b1);        
        Eigen::Vector3d getLatVec(const Eigen::Vector3d &u);
        Eigen::Vector3d calVFAccLaw();

        double computeCollectiveThrust(const Eigen::Vector3d &a_cmd, const Eigen::Vector3d &b3);
        Eigen::Vector3d limitTilt(const Eigen::Vector3d &d3);
        Eigen::Vector4d computeDesiredAttitude(const double head_des, const double head, const Eigen::Vector3d &d3);
      
        double clamp(const double x, const double x_min, const double x_max);

        void publish(const Eigen::Vector4d &qW_D, const double thr_c);
        void landed();

        std::string getTimestampFilename();
        void logging();
};

#endif