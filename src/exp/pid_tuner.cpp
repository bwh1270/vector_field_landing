#include "aims_als/exp/pid_tuner.hpp"


PIDTuner::PIDTuner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private):
nh_(nh),
nh_private_(nh_private),
hold_mode_("AUTO.LOITER"),
offboard_mode_("OFFBOARD")
{
    // set parameters
    nh_private_.param<std::string>("tuning_what",   _tuning_what, "velocity");
    nh_private_.param<std::string>("tuning_axis",   _tuning_axis, "z");
    nh_private_.param<double>("head_setpoint",      _head_sp,     90.);
    _head_sp = deg2rad(_head_sp);

    nh_private_.param<double>("step_input_size",   _step_size, 1.);
    nh_private_.param<double>("step_duration", _step_duration, 2.);

    // ROS object
    sub_odom_uav_    = nh_.subscribe("/mavros/local_position/odom", 1, &PIDTuner::uavOdomCb, this, ros::TransportHints().tcpNoDelay());
    sub_state_uav_   = nh_.subscribe("/mavros/state", 1, &PIDTuner::uavStateCb, this);
    pub_odom_cmd_    = nh_.advertise<nav_msgs::Odometry>("/aims/gv_esti", 1);
    client_setmode_  = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    timer_pid_tuner_ = nh_.createTimer(ros::Duration(1./50.), &PIDTuner::tunePID, this);

    // init. vars.
    uav_.measureAttitudeFromImu(false); 
    first_offboard_iter_ = true;
    t0_ = std::numeric_limits<double>::max();
    tf_ = t0_;
    init_position_ << 0., 0., 0.;
    
    const Eigen::Vector3d E(0., 0., _head_sp);
    const auto q = E2q(E);
    ref_odom_.pose.pose.orientation.w = q(0);
    ref_odom_.pose.pose.orientation.x = q(1);
    ref_odom_.pose.pose.orientation.y = q(2);
    ref_odom_.pose.pose.orientation.z = q(3);

    printTuningInfo();
}

void PIDTuner::printTuningInfo()
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "##---------------------------------------##" << std::endl;
    std::cout << "|| PIDTuner Node is initialized..!"          << std::endl;
	std::cout << "|| Selected: [" << _tuning_what << "]"       << std::endl;
    std::cout << "|| Head setpoint = [" << rad2deg(_head_sp) << "]" << std::endl;
    std::cout << "|| Selected: [" << _tuning_axis << "]-axis in {W}" << std::endl;
    std::cout << "|| Setpoint: [" << _step_size << "] step input during [" << _step_duration << "] seconds" << std::endl;;
    std::cout << "##---------------------------------------##" << std::endl;
}

void PIDTuner::uavStateCb(const mavros_msgs::State::ConstPtr& msg)
{
    mode_ = msg->mode;
}

void PIDTuner::uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    auto t = msg->header.stamp.toSec();
    auto p = point2vec(msg->pose.pose.position);
    auto q = quat2vec(msg->pose.pose.orientation);
    auto v = vec2vec(msg->twist.twist.linear);

    uav_.updatePX4OdomState(t, p, q, v);
}

void PIDTuner::changeMode(const std::string &mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;

    if (client_setmode_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("Mode changed to %s", mode.c_str());
    } else {
        ROS_WARN("Failed to change mode to %s", mode.c_str());
    }
}

void PIDTuner::updateInitialPosition()
{        
    for (size_t i=0; i<3; ++i) {
        init_position_(i) = uav_.p_(i);
    }
}

bool PIDTuner::measureResponse(const double t)
{
    if (mode_ == offboard_mode_) {

        if (first_offboard_iter_) {
            first_offboard_iter_ = false;
            t0_ = t;
            tf_ = t0_ + _step_duration;

            std::cout << "Initial [x,y,z] = [" << init_position_(0) << ", " 
                                               << init_position_(1) << ", " 
                                               << init_position_(2) << "]" 
                                               << std::endl;
            
            std::cout << "Response is tracking now..." << std::endl;
        }

        if (t > tf_) {
            std::cout << "Stop measuring response due to [t >= tf]" << std::endl;
            t0_ = std::numeric_limits<double>::max();
            tf_ = t0_;
            return true;
        }
        else {
            time_.push_back(t);

            if (_tuning_what == "velocity") {

                if (_tuning_axis == "x") {
                    resp_.push_back(uav_.v_(0));
                } 
                else if (_tuning_axis == "y") {
                    resp_.push_back(uav_.v_(1));
                } 
                else if (_tuning_axis == "z") {
                    resp_.push_back(uav_.v_(2));
                }
            } 
            else if (_tuning_what == "position") {
            
                if (_tuning_axis == "x") {
                    resp_.push_back(uav_.p_(0) - init_position_(0));
                }
                else if (_tuning_axis == "y") {
                    resp_.push_back(uav_.p_(1) - init_position_(1));
                }
                else if (_tuning_axis == "z") {
                    resp_.push_back(uav_.p_(2) - init_position_(2));
                }
            }

            return false;
        }
    }
    else {
        // Ready for offboard mode
        return false;
    }
}

void PIDTuner::printPlotAndHold()
{
    if ((time_.empty()) || (resp_.empty()) || (time_.size() != resp_.size())) {
        throw std::invalid_argument("Time and response vectors must be non-empty and of equal size.");
    }

    // shift the times
    double t0 = time_.front();
    for (auto &t : time_) {
        t -= t0;
    }

    // compute performance indices
    const auto N = time_.size();
    double Mp(0.), Tr(0.), Ts(0.);

    if (_tuning_what == "velocity") {

        // unify the sign
        for (auto &y : resp_) {
            y = abs(y);
        }

        // find Mp (maximum peak)
        Mp = *std::max_element(resp_.begin(), resp_.end());

        // find Tr (rise time)
        bool found_10_percent(false), found_90_percent(false);

        for (size_t i=0; i<N; ++i) {
            if ((!found_10_percent) && (resp_[i] >= (0.1 * _step_size))) {
                Tr = time_[i];
                found_10_percent = true;
            }
            if ((!found_90_percent) && (resp_[i] >= (0.9 * _step_size))) {
                Tr = time_[i] - Tr;
                found_90_percent = true;
                break;
            }
        }

        // find Ts (settling time)
        const double tolerance = 0.05;
        for (size_t i=N-1; i>=0; --i) {
            if (abs(resp_[i] - _step_size) > tolerance * _step_size) {
                Ts = time_[i+1];
                break;
            }
        }
    }
    else if (_tuning_what == "position") {

        // find Mp (maximum peak)
        Mp = *std::max_element(resp_.begin(), resp_.end());

        // find Tr (rise time)
        bool found_10_percent(false), found_90_percent(false);

        for (size_t i=0; i<N; ++i) {
            if ((!found_10_percent) && (resp_[i] >= (0.1 * _step_size))) {
                Tr = time_[i];
                found_10_percent = true;
            }
            if ((!found_90_percent) && (resp_[i] >= (0.9 * _step_size))) {
                Tr = time_[i] - Tr;
                found_90_percent = true;
                break;
            }
        }

        // find Ts (settling time)
        const double tolerance = 0.05;
        for (size_t i=N-1; i>=0; --i) {
            if (abs(resp_[i] - _step_size) > tolerance * _step_size) {
                Ts = time_[i+1];
                break;
            }
        }
    }
    
    // Print results
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "##------------------------------##"    << std::endl;
    std::cout << "|| Response results: "                 << std::endl;
    std::cout << "|| Rise Time:          [" << Tr << "]" << std::endl;
    std::cout << "|| Settling Time:      [" << Ts << "]" << std::endl;
    std::cout << "|| Maximum Peak Value: [" << Mp << "]" << std::endl;
    std::cout << "|| Last Response:      [" << resp_[N-1] << "]" << std::endl;
    std::cout << "|| Tuning process is terminated..."    << std::endl;
    std::cout << "##------------------------------##"    << std::endl;

    changeMode(hold_mode_);
}

void PIDTuner::tunePID(const ros::TimerEvent &event)
{   
    if (_tuning_what == "velocity") {

        if (_tuning_axis == "x") {
            ref_odom_.twist.twist.linear.x = _step_size;
        }
        else if (_tuning_axis == "y") {
            ref_odom_.twist.twist.linear.y = _step_size;
        }
        else if (_tuning_axis == "z") {
            ref_odom_.twist.twist.linear.z = _step_size;
        }
        else {
            ROS_WARN("Invalid _tuning_axis parameter entered: %s", _tuning_axis.c_str());
            return;
        }

        if (measureResponse(ros::Time::now().toSec())) {
            printPlotAndHold();
        }

        pub_odom_cmd_.publish(ref_odom_);


    } else if (_tuning_what == "position") {

        if (first_offboard_iter_) {
            updateInitialPosition();
        }

        if (_tuning_axis == "x") {
            ref_odom_.pose.pose.position.x = init_position_(0) + _step_size;
            ref_odom_.pose.pose.position.y = init_position_(1);
            ref_odom_.pose.pose.position.z = init_position_(2);
        }
        else if (_tuning_axis == "y") {
            ref_odom_.pose.pose.position.x = init_position_(0);
            ref_odom_.pose.pose.position.y = init_position_(1) + _step_size;
            ref_odom_.pose.pose.position.z = init_position_(2);
        }
        else if (_tuning_axis == "z") {
            ref_odom_.pose.pose.position.x = init_position_(0);
            ref_odom_.pose.pose.position.y = init_position_(1);
            ref_odom_.pose.pose.position.z = init_position_(2) + _step_size;
        }
        else {
            ROS_WARN("Invalid _tuning_axis parameter entered: %s", _tuning_axis.c_str());
            return;
        }

        if (measureResponse(ros::Time::now().toSec())) {
            printPlotAndHold();
        }

        pub_odom_cmd_.publish(ref_odom_);
    } 
    else {
        ROS_WARN("Invalid _tuning_what parameter entered: %s", _tuning_what.c_str());
        ROS_WARN("It should be 'position' or 'velocity'");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_tuner_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    auto pid_tuner_obj = std::make_unique<PIDTuner>(nh, nh_private);
    
    pid_tuner_obj->run();

    return 0;
} 
