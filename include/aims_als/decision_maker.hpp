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
 * @brief Decision Maker Class 
 * @author Woohyun Byun - imbwh@cau.ac.kr
 * @date 2025.03.21
 */
#ifndef __DECISION_MAKER__
#define __DECISION_MAKER__

#include <iostream>
#include <cassert>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "aims_als/Marker.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"

#include "aims_als/lib/util.h"
#include "aims_als/lib/filter.h"
#include "aims_als/lib/math.h"

using namespace aims_fly;


class DecisionMaker
{
    public:
        DecisionMaker(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~DecisionMaker() = default;

        void run() { ros::spin(); };

    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_fractal_detections_;
        ros::Subscriber sub_gimbal_angles_;
        ros::Subscriber sub_pose_uav_;
        ros::Subscriber sub_twist_uav_;
        ros::Subscriber sub_state_uav_;
        ros::Subscriber sub_odom_gv_;
        ros::Publisher pub_fsm_state_;
        ros::Publisher pub_led_cmd_;
        ros::ServiceClient set_mode_client_;
        ros::Timer fsm_timer_;
        ros::Timer monitor_timer_;

        // Flight Mode
        struct fm_t {  
            std::string position;
            std::string hold;
            std::string offboard;
        };

        // Finite State Machine
        enum class FSM {
            ON_GROUND,      // S0
            HOLD_POSITION,  // S1
            SEARCHING,      // S2
            TRACKING,       // S3
            LANDING,        // S4
            ERROR,          // S5
            END             // S6
        };
        
        // LED Color
        struct led_t {
            std::string red;     // (255,000,000)
            std::string orange;  // (255,165,000)
            std::string green;   // (000,255,000)
            std::string blue;    // (000,000,255)
            std::string purple;  // (000,255,255)
            std::string yellow;  // (255,255,000
        };

        // UAV
        struct uav_t {
            bool armed;
            std::string fm;
            int fsm;
            Eigen::Vector3d p;
            Eigen::Matrix3d R;
            Eigen::Vector3d v;
        };

        // GV
        struct gv_t {
            Eigen::Vector3d p;
            Eigen::Vector3d v;
            double head;
        };

        struct err_eps_t {
            double dt;    // detection failure timeout
            float roll;  // angle
            float pan;
            double r;     // range
            double s;     // speed
            int cnt;  // swing counter
            double tilt;  // swing angle
        };

        // Error Transition Conditions
        struct err_cond_t {
            int num_of_sat;  // # of satisfied conditions
            bool detected;
            double t0;
            Eigen::Vector3f gimbal_angles;
            double r;
            double s;
            int cnt;
        };

        struct vf_t {
            double phi_des;
            double phi_delta; // boundary = phi_des +- phi_delta +- phi_bound
            double phi_bound;
        };

        // Landing Conditions
        struct land_cond_t {
            double s;      // gv speed
            bool want2land;
            double t;
            double T;
        };

        // Landed Condition
        struct landed_cond_t {
            double crt_alt;
        };

        fm_t fm_;
        led_t led_;
        uav_t uav_;
        gv_t gv_;
        err_eps_t err_eps_;
        err_cond_t err_cond_;
        vf_t vf_;
        land_cond_t land_cond_;
        landed_cond_t landed_cond_;

        // PARAMS
        double _fsm_rate;
        double _monitor_rate;
        bool _sitl;

        // OUTPUT
        std_msgs::Int32 fsm_msgs_;
        std_msgs::String led_msgs_;

        void setFlightMode();
        void setLEDColor();
        void initUAV();
        void initErrCond();

        void markerCb(const aims_als::Marker::ConstPtr &msg);
        void gimbalAngleCb(const geometry_msgs::PointStamped::ConstPtr &msg);
        void uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void uavTwistCb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void uavStateCb(const mavros_msgs::State::ConstPtr &msg);
        void gvEstiCb(const nav_msgs::Odometry::ConstPtr &msg);
        void changeFlightMode(const std::string &mode);

        double computeHead(const Eigen::Vector3d &b1);        

        void fsmLoop(const ros::TimerEvent &event);
        void monitor(const ros::TimerEvent &event);

        int checkFSM();
        bool checkErrorConditions();
        bool landingConditions();
        bool landedConditions();
        std::string toString(int fsm);
};



#endif