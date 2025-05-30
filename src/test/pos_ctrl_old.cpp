#include "vector_field_landing/pos_ctrl_old.hpp"


PositionControl::PositionControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private):
nh_(nh),
nh_private_(nh_private),
eW_z_(0.,0.,1.),
g_(0., 0., 9.81)
{
    // Init parameters
    nh_private_.param<bool>("with_position", _with_pos, false);
    nh_private_.param<bool>("for_gvf",        _for_gvf, false);
    nh_private_.param<double>("const_dt",        _dt_c, 1./50.);
   
    nh_private_.param<double>("lpf_acc_alpha", _lpf_acc_alpha, 0.1);
    nh_private_.param<double>("lpf_ref_alpha", _lpf_ref_alpha, 1.0);
    nh_private_.param<double>("lpf_vel_alpha", _lpf_vel_alpha, 1.0);
    
    nh_private_.param<double>("hover_thrust", _thr_hover_init, 0.40);

    nh_private_.param<double>("vel_xy_max", _vel_xy_max, 3.0);  
    nh_private_.param<double>("vel_z_max",   _vel_z_max, 1.0);

    nh_private_.param<double>("integral_vel_xy_max", _int_vel_xy_max, 5.);
    nh_private_.param<double>("integral_vel_z_max",   _int_vel_z_max, 5.);

    nh_private_.param<double>("thrust_min", _thr_min, 0.15); 
    nh_private_.param<double>("thrust_max", _thr_max, 0.55);

    nh_private_.param<double>("tilt_max", _tilt_max, 40.);
    _tilt_max = deg2rad(_tilt_max);


    Eigen::Vector3d P,I,D;
    nh_private_.param<double>("px_kp", P(0), 1.);
    nh_private_.param<double>("px_kd", D(0), 0.);

    nh_private_.param<double>("py_kp", P(1), 1.);
    nh_private_.param<double>("py_kd", D(1), 0.);

    nh_private_.param<double>("pz_kp", P(2), 1.);
    nh_private_.param<double>("pz_kd", D(2), 0.);

    P_.P = P.asDiagonal();
    P_.D = D.asDiagonal();

    nh_private_.param<double>("vx_kp", P(0), 3.);
    nh_private_.param<double>("vx_ki", I(0), 4.);
    nh_private_.param<double>("vx_kd", D(0), 0.2);

    nh_private_.param<double>("vy_kp", P(1), 3.);
    nh_private_.param<double>("vy_ki", I(1), 4.);
    nh_private_.param<double>("vy_kd", D(1), 0.2);

    nh_private_.param<double>("vz_kp", P(2), 3.);
    nh_private_.param<double>("vz_ki", I(2), 4.);
    nh_private_.param<double>("vz_kd", D(2), 0.2);

    V_.P = P.asDiagonal();
    V_.I = I.asDiagonal();
    V_.D = D.asDiagonal();


    sub_odom_uav_   = nh_.subscribe("/mavros/local_position/odom", 1, &PositionControl::uavOdomCb, this, ros::TransportHints().tcpNoDelay());
    sub_imu_uav_    = nh_.subscribe("/mavros/imu/data", 1, &PositionControl::uavImuCb, this, ros::TransportHints().tcpNoDelay());
    sub_state_uav_  = nh_.subscribe("/mavros/state", 1, &PositionControl::uavStateCb, this);
    sub_hover_thrust_ = nh_.subscribe("/mavros/hover_thrust_estimate/hover_thrust", 1, &PositionControl::hoverThrustCb, this, ros::TransportHints().tcpNoDelay());
    sub_odom_esti_  = nh_.subscribe("/aims/gv_esti", 1, &PositionControl::gvEstiOdomCb, this, ros::TransportHints().tcpNoDelay());
    sub_bool_land_  = nh_.subscribe("/aims/land", 1, &PositionControl::landCb, this, ros::TransportHints().tcpNoDelay());
    pub_attiTarget_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);


    // Init control states
    uav_.measureAttitudeFromImu(true);
    uav_sp_.measureAttitudeFromImu(false);

    uav_.setLpfAccAlpha(_lpf_acc_alpha);
    uav_.setLpfVelAlpha(_lpf_vel_alpha);
    uav_sp_.setLpfVelAlpha(_lpf_ref_alpha); 
    
    asp_ = g_;
    qW_D_ << 1., 0., 0., 0.;
    int_vel_ << 0., 0., 0.;

    land_ = false;

    ROS_INFO("Position Control Node is initialized...");
}


void PositionControl::uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    const double t = msg->header.stamp.toSec();
    const Eigen::Vector3d p = point2vec(msg->pose.pose.position);
    const Eigen::Vector4d q = quat2vec(msg->pose.pose.orientation);
    const Eigen::Vector3d v = vec2vec(msg->twist.twist.linear);

    uav_.updatePX4OdomState(t, p, q, v);
    // uav_.stateMonitoring();
}

void PositionControl::uavStateCb(const mavros_msgs::State::ConstPtr& msg)
{
    mode_ = msg->mode;
}

void PositionControl::landCb(const std_msgs::Bool::ConstPtr &msg)
{
    land_ = msg->data;
}

void PositionControl::hoverThrustCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    const bool valid = msg->pose.position.z;

    if (valid) {
        thr_hover_esti_ = msg->pose.position.x;
    }
    else {
        thr_hover_esti_ = _thr_hover_init;
    }
}

void PositionControl::gvEstiOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    const double t = ros::Time::now().toSec();
    const Eigen::Vector3d p = point2vec(msg->pose.pose.position);
    const Eigen::Vector4d q = quat2vec(msg->pose.pose.orientation);
    const Eigen::Vector3d v = vec2vec(msg->twist.twist.linear);

    double dt = t - uav_sp_.t_.odom;
    if (dt <= N_EPS) { 
        dt = _dt_c; 
        ROS_WARN("dt is too small");
    } else if (dt > 1.) {
        dt = 0.;
        ROS_WARN("dt is too large");
    }

    uav_sp_.updateOdomState(t, p, q, v);


    // Position and velocity control (sample time: k)
    if (_for_gvf) {
        // gv_esti = [x_W, y_W, z_W, head_WM, vx_W, vy_W, vz_W]
        velCtrl(uav_sp_, uav_, dt);

    } else {
        if (_with_pos) {
            posCtrl(uav_sp_, uav_);
        }
        velCtrl(uav_sp_, uav_, dt);
    }
}

/**
 * @brief PD with Filtered Feedforward
 */
void PositionControl::posCtrl(UAV &uav_sp, const UAV &uav)
{
    // (1) compute the error
    const Eigen::Vector3d err_p = uav_sp.p_ - uav.p_;

    // (2) compute the control law
    const Eigen::Vector3d vsp = P_.P * err_p + P_.D * (-uav.v_) + uav_sp.v_; 

    // (3) treat the saturation
    Eigen::Vector2d vsp_xy(vsp(0), vsp(1));
    double vsp_z = vsp(2);
    
    if (vsp_xy.norm() >= _vel_xy_max) {
        vsp_xy = vsp_xy * (_vel_xy_max / vsp_xy.norm());
    }

    if (abs(vsp_z) >= _vel_z_max) {
        vsp_z *= _vel_z_max / abs(vsp_z);
    }

    uav_sp.v_(0) = vsp_xy(0);
    uav_sp.v_(1) = vsp_xy(1);
    uav_sp.v_(2) = vsp_z;
}

/** 
 * @brief PID with Anti-windup
 **/
void PositionControl::velCtrl(UAV &uav_sp, const UAV &uav, const double dt)
{
    // (1) compute the error 
    const Eigen::Vector3d err_v = uav_sp.v_ - uav.v_;

    // (2) compute the control law
    asp_ = V_.P * err_v + int_vel_ + V_.D * (-uav.a_) + g_;
    std::cout << "P: " << (V_.P * err_v).transpose() << std::endl;
    std::cout << "I: " << (int_vel_).transpose() << std::endl;
    std::cout << "D: " << (V_.D * (-uav.a_)).transpose() << std::endl;
    
    // (3) compute the desired thrust vector and desired attitude @ 50 Hz
    const double Tc = computeCollectiveThrust(asp_, uav_.eB_z_);

    const Eigen::Vector3d eD_z = (asp_).normalized();

    const Eigen::Vector3d eD_z_lim = limitTilt(eD_z);

    qW_D_ = computeDesiredAttitude(uav_sp_.head_, uav_.head_, eD_z_lim);

    // (4) integral clamping
    if (mode_ != "OFFBOARD") {
        int_vel_ << 0., 0., 0.;
    
    } else {
        int_vel_ += (V_.I * err_v) * dt;
        
        // anti-windup clamping
        for (int i=0; i<2; ++i) {
            int_vel_(i) = max(-_int_vel_xy_max, min(int_vel_(i), _int_vel_xy_max)); 
        }
        int_vel_(2) = max(-_int_vel_z_max, min(int_vel_(2), _int_vel_z_max));
    }
}

double PositionControl::computeCollectiveThrust(const Eigen::Vector3d &asp_k, const Eigen::Vector3d &eB_z)
{
    // convert acceleration to normalized collective thrust
    double Tc = asp_k.dot(eB_z) * thr_hover_esti_ / g_.norm();

    // saturation for safety
    if (Tc >= _thr_max) {
        Tc = _thr_max;
    }
    else if (Tc <= _thr_min) {
        Tc = _thr_min;
    }

    return Tc;
}

Eigen::Vector3d PositionControl::limitTilt(const Eigen::Vector3d &eD_z)
{
    Eigen::Vector3d eD_z_lim = eD_z;
    const double tilt = acos(eD_z.dot(eW_z_));
    
    if (tilt >= _tilt_max) {
        const double lambda = sin(_tilt_max) / sqrt(pow(eD_z(0),2) + pow(eD_z(1),2));
        
        eD_z_lim(0) *= lambda;
        eD_z_lim(1) *= lambda;
        eD_z_lim(2) = cos(_tilt_max); 
    }

    return eD_z_lim;
}


Eigen::Vector4d PositionControl::computeDesiredAttitude(const double head_sp, const double head, const Eigen::Vector3d &eD_z)
{
    // Compute the Rot_W_D
    const double lambda = sqrt(pow(eD_z(2),2) / (pow(eD_z(2),2) + pow(cos(head_sp) * eD_z(0) + sin(head_sp) * eD_z(1), 2)));

    // Check for NaN
    if (std::isnan(head_sp) || std::isnan(head) || std::isnan(lambda))
    {
        throw std::runtime_error(
            "Error in computeDesiredAttitude: head_sp, head, or lambda is NaN."
        );
    }
    
    const Eigen::Vector3d eD_x(lambda*cos(head_sp), lambda*sin(head_sp), -lambda*(cos(head_sp) * eD_z(0) + sin(head_sp) * eD_z(1))/eD_z(2));
    
    const Eigen::Vector3d eD_y = eD_z.cross(eD_x);
    
    Eigen::Matrix3d R;
    R << eD_x, eD_y, eD_z;

    return R2q(R);
}

void PositionControl::uavImuCb(const sensor_msgs::Imu::ConstPtr &msg)
{
    const double t = msg->header.stamp.toSec();
    const Eigen::Vector3d a = vec2vec(msg->linear_acceleration);
    const Eigen::Vector4d q = quat2vec(msg->orientation);

    uav_.updateImuState(t, a, q);

    // Acceleration control (sample time: t)
    accCtrl();
}

void PositionControl::accCtrl()
{
    // Compute the desired collective thrust @ 200 Hz
    double Tc = computeCollectiveThrust(asp_, uav_.eB_z_);
    
    if (land_) {
        double Tc = 0.;
        return;
    }

    // publish
    publish(qW_D_, Tc);
}

void PositionControl::publish(const Eigen::Vector4d &qW_D, const double Tc)
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.type_mask = 1+2+4;
    msg.orientation.w = qW_D(0);
    msg.orientation.x = qW_D(1);
    msg.orientation.y = qW_D(2);
    msg.orientation.z = qW_D(3);
    msg.thrust = Tc;
    pub_attiTarget_.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_ctrl_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    auto pos_ctrl_obj = std::make_unique<PositionControl>(nh, nh_private);
    
    pos_ctrl_obj->run();

    return 0;
} 
