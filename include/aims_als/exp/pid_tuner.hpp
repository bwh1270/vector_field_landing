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
 * @brief Computing Performance indices w.r.t. step reference inputs
 * @author Woohyun Byun - imbwh@cau.ac.kr
 * @date 2024.11.17
 */
#ifndef __PID_TUNER__
#define __PID_TUNER__

#include <iostream>
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
#include "mavros_msgs/State.h"
// Output Types
#include "mavros_msgs/SetMode.h"

#include "aims_als/lib/util.h"
#include "aims_als/lib/math.h"
#include "aims_als/lib/uav.h"

using namespace aims_fly;

struct limit_t {
    double x;
    double y;
    double z;
};
class PIDTuner
{
    public:
        PIDTuner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~PIDTuner() = default;

        void run() { ros::spin(); };


    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_odom_uav_;
        ros::Publisher  pub_odom_cmd_;
        ros::Subscriber sub_state_uav_;
        ros::ServiceClient client_setmode_;
        ros::Timer timer_pid_tuner_;

        UAV uav_;
        std::string mode_;
        nav_msgs::Odometry ref_odom_;

        bool first_offboard_iter_;
        double t0_, tf_;
        Eigen::Vector3d init_position_;  // only for position

        vector<double> time_;
        vector<double> resp_;

        // Constants
        const string hold_mode_;
        const string offboard_mode_;

        // Parameters
        std::string _tuning_what;
        std::string _tuning_axis;
        double _head_sp;

        double _step_size;
        double _step_duration;


        // Callback Functions
        void uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
        void uavStateCb(const mavros_msgs::State::ConstPtr& msg);
        void tunePID(const ros::TimerEvent &event);

        // Functions
        void printTuningInfo();
        void changeMode(const std::string &mode);
        void updateInitialPosition();
        bool measureResponse(const double t);
        void printPlotAndHold(); 

};

#endif