#include "aims_als/decision_maker.hpp"



DecisionMaker::DecisionMaker(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
nh_(nh), nh_private_(nh_private)
{
    
    nh_private_.param<double>("Hz_of_FSM", _fsm_rate, 50.0);
    nh_private_.param<double>("Hz_of_MONITOR", _monitor_rate, 1.0);
    nh_private_.param<bool>("sitl", _sitl, false);

    nh_private_.param<double>("epsilon_dt", err_eps_.dt, 0.1);
    nh_private_.param<float>("epsilon_angle", err_eps_.angle, 10.f);
    nh_private_.param<double>("epsilon_range", err_eps_.r, 5.0);
    nh_private_.param<double>("epsilon_speed", err_eps_.s, 4.0);
    nh_private_.param<int>("epsilon_swing_cnt", err_eps_.cnt, 3);
    nh_private_.param<double>("epsilon_swing_angle", err_eps_.tilt, 40.);    

    nh_private_.param<double>("vf_phi_des", vf_.phi_des, -20.0);
    nh_private_.param<double>("vf_phi_delta", vf_.phi_delta, 40.0);
    vf_.phi_des = aims_fly::deg2rad(vf_.phi_des);
    vf_.phi_delta = aims_fly::deg2rad(vf_.phi_delta);
    
    nh_private_.param<double>("landing_condition_gv_speed", land_cond_.s, 5.0);
    nh_private_.param<double>("landing_condition_duration", land_cond_.T, 3.0);
    land_cond_.want2land = false;
    land_cond_.t = 0.;

    nh_private_.param<double>("landed_condition_critical_altitude", landed_cond_.crt_alt, 0.2);


    // Set Constants
    setFlightMode();
    setLEDColor();
    initUAV();
    initErrCond();

    // ROS
    sub_fractal_detections_ = nh_.subscribe("/aims/fractal_detections", 1, &DecisionMaker::markerCb, this, ros::TransportHints().tcpNoDelay());
    sub_gimbal_angles_      = nh_.subscribe("/aims/gimbal_angles", 1, &DecisionMaker::gimbalAngleCb, this, ros::TransportHints().tcpNoDelay());
    sub_pose_uav_           = nh_.subscribe("/mavros/local_position/pose", 1, &DecisionMaker::uavPoseCb, this, ros::TransportHints().tcpNoDelay());
    sub_twist_uav_          = nh_.subscribe("/mavros/local_position/velocity_local", 1, &DecisionMaker::uavTwistCb, this, ros::TransportHints().tcpNoDelay());
    sub_state_uav_          = nh_.subscribe("/mavros/state", 1, &DecisionMaker::uavStateCb, this, ros::TransportHints().tcpNoDelay());
    if (_sitl)
    { sub_odom_gv_          = nh_.subscribe("/odom", 1, &DecisionMaker::gvEstiCb, this, ros::TransportHints().tcpNoDelay()); }
    else 
    { sub_odom_gv_          = nh_.subscribe("/aims/gv_esti", 1, &DecisionMaker::gvEstiCb, this, ros::TransportHints().tcpNoDelay()); }
    pub_fsm_state_ = nh_.advertise<std_msgs::Int32>("/aims/fsm_state", 1);
    pub_led_cmd_   = nh_.advertise<std_msgs::String>("/aims/led_color_cmd", 1);
    set_mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    fsm_timer_     = nh_.createTimer(ros::Duration(1./_fsm_rate), &DecisionMaker::fsmLoop, this);
    monitor_timer_ = nh_.createTimer(ros::Duration(1./_monitor_rate), &DecisionMaker::monitor, this);
}

void DecisionMaker::setFlightMode()
{
    fm_.position = "POSCTL";
    fm_.hold = "AUTO.LOITER";
    fm_.offboard = "OFFBOARD";
}

void DecisionMaker::setLEDColor()
{
    led_.red = "255,0,0";
    led_.orange = "255,165,0";
    led_.green = "0,255,0";
    led_.blue = "0,0,255";
    led_.purple = "0,255,255";
    led_.yellow = "255,255,0";
}

void DecisionMaker::initUAV()
{
    uav_.armed = false;
    uav_.fm = fm_.position;
    uav_.fsm = static_cast<int>(FSM::ON_GROUND);
    uav_.p << 0., 0., 0.;
    uav_.R = Eigen::Matrix3d::Identity();
    uav_.v << 0., 0., 0.;
}

void DecisionMaker::initErrCond()
{
    err_cond_.num_of_sat = 0;
    err_cond_.detected = false;
    err_cond_.t0 = 0.;
    err_cond_.gimbal_angles << 0.f, 0.f;
    err_cond_.r = 0.;
    err_cond_.s = 0.;
    err_cond_.cnt = 0;
}

void DecisionMaker::markerCb(const aims_als::Marker::ConstPtr &msg)
{
    err_cond_.detected = msg->detect;
    err_cond_.t0 = ros::Time::now().toSec();
}

void DecisionMaker::gimbalAngleCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    err_cond_.gimbal_angles(0) = msg->point.x;  // roll
    err_cond_.gimbal_angles(1) = msg->point.y;  // pitch(tilt)
}

void DecisionMaker::uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_.p = point2vec(msg->pose.position);
    const Eigen::Vector4d q = quat2vec(msg->pose.orientation);
    uav_.R = q2R(q);
}

void DecisionMaker::uavTwistCb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_.v = vec2vec(msg->twist.linear);
}

void DecisionMaker::uavStateCb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_.armed = msg->armed;
    uav_.fm = msg->mode;
}

void DecisionMaker::gvEstiCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    gv_.p = point2vec(msg->pose.pose.position);
    gv_.v = vec2vec(msg->twist.twist.linear);

    const Eigen::Vector4d q = quat2vec(msg->pose.pose.orientation);
    const Eigen::Matrix3d R = q2R(q);
    gv_.head = computeHead(R.col(0));
}

void DecisionMaker::changeFlightMode(const std::string &mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;

    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("Mode changed to %s", mode.c_str());
    } else {
        ROS_WARN("Failed to change mode to %s", mode.c_str());
    }
}

double DecisionMaker::computeHead(const Eigen::Vector3d &b1)
{
    const Eigen::Vector2d b1_e12(b1(0), b1(1));
    const double head = acos(b1_e12.dot(Eigen::Vector2d(1.,0.)) / b1_e12.norm());
    
    // 1&2 quadrant -> head>0
    // 3&4 quadrant -> head<0
    if (b1(1) >= 0) {
        return head;
    } else {
        return -head;
    }
}

void DecisionMaker::fsmLoop(const ros::TimerEvent &event)
{

// Offboard && ErrorConditions => S5: Error
    const int fsm = checkFSM();
    led_msgs_.data = led_.red;
    switch (fsm)
    {
        case static_cast<int>(FSM::ON_GROUND):
            if ((uav_.armed) && ((uav_.fm == fm_.position) || (uav_.fm == fm_.hold))) {
                uav_.fsm = static_cast<int>(FSM::HOLD_POSITION);
            }
            break;
        

        case static_cast<int>(FSM::HOLD_POSITION): 
            if (!checkErrorConditions()) {
                assert(err_cond_.num_of_sat >= 0);
                if (err_cond_.num_of_sat >= 1) {
                    uav_.fsm = static_cast<int>(FSM::SEARCHING);
                    led_msgs_.data = led_.orange;
                }
            } 
            else {
                uav_.fsm = static_cast<int>(FSM::SEARCHING);
                led_msgs_.data = led_.green;
            }
            break;

        case static_cast<int>(FSM::SEARCHING):
            if (checkErrorConditions()) {
                assert(err_cond_.num_of_sat == 5);
                led_msgs_.data = led_.green;
                if (uav_.fm == fm_.offboard) {
                    uav_.fsm = static_cast<int>(FSM::TRACKING);
                    led_msgs_.data = led_.blue;
                }
            } 
            else {
                if (err_cond_.num_of_sat == 0) {
                    uav_.fsm == static_cast<int>(FSM::HOLD_POSITION);
                    led_msgs_.data = led_.red;
                }
            }
            break;
        
        case static_cast<int>(FSM::TRACKING):
            if (uav_.fm == fm_.offboard) {
                if (checkErrorConditions()) {
                    led_msgs_.data = led_.blue;
                    if (landingConditions()) {
                        uav_.fsm = static_cast<int>(FSM::LANDING);
                        led_msgs_.data = led_.purple;
                    }
                }
                else {
                    uav_.fsm = static_cast<int>(FSM::ERROR);
                    led_msgs_.data = led_.red;
                }
            } 
            else {
                uav_.fsm = static_cast<int>(FSM::HOLD_POSITION);
                led_msgs_.data = led_.red;
            }
            break;

        case static_cast<int>(FSM::ERROR):
            changeFlightMode(fm_.hold);
            ros::Duration(2.0).sleep();
            initUAV();
            initErrCond();
            uav_.fsm = static_cast<int>(FSM::HOLD_POSITION);
            led_msgs_.data = led_.red;
            break;

        case static_cast<int>(FSM::LANDING):
            if (uav_.fm == fm_.offboard) {
                if (checkErrorConditions()) {
                    led_msgs_.data = led_.purple;
                    if (landedConditions()) {
                        uav_.fsm = static_cast<int>(FSM::END);
                        led_msgs_.data = led_.yellow;
                    }
                }
                else {
                    uav_.fsm = static_cast<int>(FSM::ERROR);
                    led_msgs_.data = led_.red;
                }
            } 
            else {
                uav_.fsm = static_cast<int>(FSM::HOLD_POSITION);
                led_msgs_.data = led_.red;
            }
            break;

        case static_cast<int>(FSM::END):
            led_msgs_.data = led_.yellow;
            ROS_INFO("Mission is reached: END State");
            fsm_timer_.stop();
            break;
        
        default:
            ROS_ERROR("Not Expected State!!");
            led_msgs_.data = led_.red;
    }

    fsm_msgs_.data = checkFSM();

    pub_fsm_state_.publish(fsm_msgs_);
    pub_led_cmd_.publish(led_msgs_);
}

int DecisionMaker::checkFSM()
{
    return static_cast<int>(uav_.fsm);
}

bool DecisionMaker::checkErrorConditions()
{
    err_cond_.num_of_sat = 0;

    // 1. Detection timeout
    const double t = ros::Time::now().toSec();
    const double dt = t - err_cond_.t0;
    assert(dt>=0);

    if (dt > err_eps_.dt) {
        ROS_WARN("Detection Timeout");
        return false;
    }
    ++err_cond_.num_of_sat;

    // 2. Gimbal shutdown [deg]
    const float diff_angle = abs(err_cond_.gimbal_angles(0));
    assert(diff_angle>=0);

    if (diff_angle > err_eps_.angle) {
        ROS_WARN("Gimbal might be shutdown");
        return false;
    }
    ++err_cond_.num_of_sat;

    // 3. Over Maximum Range and Speed
    const double r = (gv_.p - uav_.p).norm();
    const double s = (gv_.v - uav_.v).norm();
    assert((r>=0) && (s>=0));

    if ((r > err_eps_.r) || (s > err_eps_.s)) {
        ROS_WARN("Over Maximum Range or Speed");
        return false;
    }
    ++err_cond_.num_of_sat;

    // 4. Large Swing Counter [rad]
    const double tilt = acos(uav_.R(2,2));

    if (tilt > err_eps_.tilt) {
        ++err_cond_.cnt; 
    } else {
        err_cond_.cnt = 0;
    }

    if (err_cond_.cnt > err_eps_.cnt) {
        ROS_WARN("Dangerous Swings detected");
        return false;
    }
    ++err_cond_.num_of_sat;

    // 5. Out of Vector Field Detectable Set
    // radius is already considered in (3)
    // Eigen::MatrixXd M;
    // M.resize(2,3);
    // M << 1., 0., 0.,
    //      0., 1., 0.;
    // const Eigen::Vector2d rel_pos_xy = M * (uav_.p - gv_.p);
    // const double rel_pos_xy_lon = rel_pos_xy.dot(Eigen::Vector2d(cos(gv_.head), sin(gv_.head)));
    // const double phi = atan2(rel_pos_xy_lon, uav_.p(2)-gv_.p(2));
    
    // if ((phi > (vf_.phi_des + vf_.phi_delta)) || (phi < (vf_.phi_des - vf_.phi_delta))) {
    //     // ROS_DEBUG("Out of Detectable Set: [%.2f, (%.2f,f, %.2f)]", phi, vf_.phi_des + vf_.phi_delta, vf_.phi_des - vf_.phi_delta);
    //     return false;
    // }
    ++err_cond_.num_of_sat;

    return true;
}

bool DecisionMaker::landingConditions()
{
    if (!land_cond_.want2land) {
        land_cond_.t = ros::Time::now().toSec();
        land_cond_.want2land = true;
    } else {
        const double dt = ros::Time::now().toSec() - land_cond_.t;
        if (dt > land_cond_.T) {
            return true;
        }
    }

    // GV speed reaches the setpoint    
    if (gv_.v.norm() > land_cond_.s) {
        return true;
    }

    return false;
}

bool DecisionMaker::landedConditions()
{
    // rel_vel?, rel_pos?
    const double dz = uav_.p(2) - gv_.p(2);

    if ((abs(dz) < landed_cond_.crt_alt) && (!uav_.armed)) {
        return true;
    }
    // if (abs(dz) < landed_cond_.crt_alt) {
    //     return true;
    // }

    return false;
}

void DecisionMaker::monitor(const ros::TimerEvent &event)
{
    ROS_INFO("Current FSM State: [%s]", toString(uav_.fsm).c_str());
    ROS_INFO("Current MAV State: [%s]", uav_.fm.c_str());
}

std::string DecisionMaker::toString(int fsm) {
    switch (fsm) {
        case static_cast<int>(FSM::ON_GROUND):      return "ON_GROUND";
        case static_cast<int>(FSM::HOLD_POSITION):  return "HOLD_POSITION";
        case static_cast<int>(FSM::SEARCHING):      return "SEARCHING";
        case static_cast<int>(FSM::TRACKING):       return "TRACKING";
        case static_cast<int>(FSM::LANDING):        return "LANDING";
        case static_cast<int>(FSM::ERROR):          return "ERROR";
        case static_cast<int>(FSM::END):            return "END";
        default:                                    return "LOGIC_BROKEN";
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_maker_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    auto decision_maker_obj = std::make_unique<DecisionMaker>(nh, nh_private);
    
    decision_maker_obj->run();


    return 0;
}