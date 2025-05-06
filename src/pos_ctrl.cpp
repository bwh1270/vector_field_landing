#include "aims_als/pos_ctrl.hpp"


PositionControl::PositionControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private):
nh_(nh), nh_private_(nh_private),
w3_(0.,0.,1.), g_(0., 0., 9.81),
MAX_SIZE(20)
{
    nh_private_.param<double>("Hz_of_MONITOR", _monitor_rate, 1.0);

    nh_private_.param<bool>("feedforward", flag_.ff, false);
    nh_private_.param<bool>("DOB", flag_.dob, false);
    nh_private_.param<bool>("debug", flag_.debug, false);
    nh_private_.param<bool>("sitl", flag_.sitl, false);
    nh_private_.param<bool>("logging", flag_.log, false);
    nh_private_.param<bool>("fixed_yaw", flag_.fixed_yaw, false);

    nh_private_.param<double>("uav_vel_lpf_freq", uav_.v_lpf_freq, 1.0);
    nh_private_.param<double>("uav_acc_lpf_freq", uav_.a_lpf_freq, 1.0);
    nh_private_.param<double>("uav_hover_thrust", _thr_h_const, 0.40);

    nh_private_.param<double>("gv_vel_lpf_freq", gv_.v_lpf_freq, 1.5);
    nh_private_.param<double>("gv_head_lpf_freq", gv_.h_lpf_freq, 5.0);

    nh_private_.param<double>("standoff_x", cmd_.standoff(0), 0.0);
    nh_private_.param<double>("standoff_y", cmd_.standoff(1), 0.0);
    nh_private_.param<double>("standoff_z", cmd_.standoff(2), 2.0);
    nh_private_.param<double>("a_cmd_alpha", cmd_.a_cmd_alpha, 1.0);
    assert(cmd_.standoff(0) <= 0.0);

    nh_private_.param<double>("q_filter_lpf_freq", dob_.q_lpf_freq, 0.01);
    nh_private_.param<double>("time_constant_of_uav", dob_.tau_uav, 1.0);

    nh_private_.param<bool>("vf_hyper", vf_.hyper, false);
    nh_private_.param<double>("vf_phi_des", vf_.phi_des, -20.0);
    nh_private_.param<double>("vf_phi_delta", vf_.phi_delta, 40.0);
    nh_private_.param<double>("vf_r_max", vf_.r_max, 5.0);
    nh_private_.param<double>("vf_k1", vf_.k1, 1.0);
    nh_private_.param<double>("vf_k2", vf_.k2, 1.0);
    nh_private_.param<double>("vf_margin", vf_.margin, 17.1888);
    nh_private_.param<double>("vf_c1", vf_.c1, 5.0);
    nh_private_.param<double>("vf_c2", vf_.c2, 2.0);
    nh_private_.param<double>("vf_n1", vf_.n1, 4.0);
    nh_private_.param<double>("vf_n2", vf_.n2, 3.0);
    nh_private_.param<double>("vf_gamma", vf_.gamma, 1.0);
    nh_private_.param<bool>("vf_moving", vf_.moving, false);
    vf_.phi_des = aims_fly::deg2rad(vf_.phi_des);
    vf_.phi_delta = aims_fly::deg2rad(vf_.phi_delta);
    // vf_.c1 = aims_fly::deg2rad(vf_.c1);
    vf_.margin = aims_fly::deg2rad(vf_.margin);
    vf_.c1 = vf_.margin / 1.2318;
    vf_.c2 = 2.65 / vf_.margin;

    if (vf_.hyper) {
        vf_.n1 = vf_.n2;
        vf_.c2 = atanh(0.90) * vf_.margin;
        vf_.c1 = abs(vf_.phi_delta) / asech(0.10);
        ROS_INFO("New Desinged Vector Field");
    }

    nh_private_.param<double>("saturation_thrust_min", sat_.thr(0), 0.25);
    nh_private_.param<double>("saturation_thrust_max", sat_.thr(1), 0.65);
    nh_private_.param<double>("saturation_tilt", sat_.tilt, 40.);
    sat_.tilt = aims_fly::deg2rad(sat_.tilt);
    nh_private_.param<double>("saturation_const_dt", sat_.dt, 1./50.);
    nh_private_.param<double>("saturation_vel_xy_max", sat_.vel_xy, 3.0);
    nh_private_.param<double>("saturation_vel_z_max", sat_.vel_z, 1.0);
    nh_private_.param<double>("saturation_integral_vel_xy_max", sat_.int_vxy, 5.0);
    nh_private_.param<double>("saturation_integral_vel_z_max", sat_.int_vz, 5.0);
    nh_private_.param<double>("saturation_no_meas_duration", sat_.T, 0.1);
    nh_private_.param<double>("saturation_critical_altitude", sat_.crt_alt, 0.35);
    nh_private_.param<double>("saturation_acc_ratio_for_land", sat_.acc_ratio, 0.5);
    assert(sat_.acc_ratio < 0.6);
    nh_private_.param<double>("saturation_land_point_altitude", sat_.land_pnt_alt, 0.0);
    
    if (sat_.land_pnt_alt >= 0.0) {
        sat_.land_pnt_lon = 0.0;
    } else {
        sat_.land_pnt_lon = abs(sat_.land_pnt_alt * tan(abs(vf_.phi_des)));
        ROS_INFO("Landing Point: [LON, ALT] = [%.2f, %.2f]", sat_.land_pnt_lon, sat_.land_pnt_alt);
    }
    

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

    // Init
    setFlightMode();
    initCmd();
    initSys();
    initDOB();
    flag_.landed = false;    

    if (flag_.log) {
        const std::string filename = getTimestampFilename();
        if (flag_.sitl) {
            log_.open("/home/bwh/" + filename);
        } else {
            log_.open("/home/aims/log/" + filename);
        }
        
        if (!log_.is_open()) {
            std::cerr << "Can not open file: " << filename << std::endl;
        } else {
            // header
            ROS_INFO("File is opened: %s", filename.c_str());
            log_ << "time,uav_p_x,uav_p_y,uav_p_z,uav_v_x,uav_v_y,uav_v_z,uav_a_x,uav_a_y,uav_a_z,uav_head,gv_p_x,gv_p_y,gv_p_z,gv_v_x,gv_v_y,gv_v_z,gv_head,cmd_v_x,cmd_v_y,cmd_v_z,cmd_int_v_x,cmd_int_v_y,cmd_int_v_z,cmd_a_x,cmd_a_y,cmd_a_z,uav_b3_e1,uav_b3_e2,uav_b3_e3,vf_h1,vf_h2\n";
        }
    }

    // ROS
    sub_pose_uav_     = nh_.subscribe("/mavros/local_position/pose", 1, &PositionControl::uavPoseCb, this, ros::TransportHints().tcpNoDelay());
    sub_twist_uav_    = nh_.subscribe("/mavros/local_position/velocity_local", 1, &PositionControl::uavTwistCb, this, ros::TransportHints().tcpNoDelay());
    sub_imu_uav_      = nh_.subscribe("/mavros/imu/data", 1, &PositionControl::uavImuCb, this, ros::TransportHints().tcpNoDelay());
    sub_fm_uav_       = nh_.subscribe("/mavros/state", 1, &PositionControl::uavFmCb, this, ros::TransportHints().tcpNoDelay());
    sub_hover_thrust_ = nh_.subscribe("/mavros/hover_thrust_estimate/hover_thrust", 1, &PositionControl::hoverThrustCb, this, ros::TransportHints().tcpNoDelay());
    
    if (flag_.sitl) 
    { sub_odom_gv_   = nh_.subscribe("/odom", 1, &PositionControl::gvEstiCb, this, ros::TransportHints().tcpNoDelay());}
    else
    { sub_odom_gv_   = nh_.subscribe("/aims/gv_esti", 1, &PositionControl::gvEstiCb, this, ros::TransportHints().tcpNoDelay()); }
    sub_fsm_state_   = nh_.subscribe("/aims/fsm_state", 1, &PositionControl::fsmCb, this, ros::TransportHints().tcpNoDelay());

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    
    pub_attitarget_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    
    monitor_timer_ = nh_.createTimer(ros::Duration(1./_monitor_rate), &PositionControl::monitor, this);

    ROS_INFO("Position Control Node is initialized...");
}

void PositionControl::setFlightMode()
{
    fm_.position = "POSCTL";
    fm_.hold = "AUTO.LOITER";
    fm_.offboard = "OFFBOARD";
}

void PositionControl::initCmd()
{
    cmd_.a = g_;
    cmd_.q << 1., 0., 0., 0.;
    cmd_.int_v << 0., 0., 0.;   
}

void PositionControl::initSys()
{
    uav_.t = 0.;
    uav_.fsm_time = 0.;
    uav_.p << 0., 0., 0.;
    uav_.v << 0., 0., 0.;
    uav_.v_prev << 0., 0., 0.;
    uav_.head = 0.;

    uav_.v_lpf_alpha = (sat_.dt / (sat_.dt + (1./(2.*aims_fly::PI*uav_.v_lpf_freq))));
    uav_.a_lpf_alpha = (sat_.dt / (sat_.dt + (1./(2.*aims_fly::PI*uav_.a_lpf_freq))));

    gv_.p << 0., 0., 0.;
    gv_.v << 0., 0., 0.;
    gv_.head = 0.;
    gv_.R << 1., 0.,
             0., 1.;

    gv_.h_lpf_alpha = (sat_.dt / (sat_.dt + (1./(2.*aims_fly::PI*gv_.h_lpf_freq))));
    gv_.v_lpf_alpha = (sat_.dt / (sat_.dt + (1./(2.*aims_fly::PI*gv_.v_lpf_freq))));
    // std::cout << gv_.h_lpf_alpha << std::endl;
    // std::cout << gv_.v_lpf_alpha << std::endl;
}

void PositionControl::initDOB()
{
    dob_.w << 0., 0., 0.;   
    dob_.u_dob_prev << 0., 0., 0.;   

    // Q(s) = a / (s + a)
    // Q(z) = (b0(z + 1) / (z + a0))
    const double a = 2.0 * aims_fly::PI * dob_.q_lpf_freq;
    const double aTs = a * sat_.dt;
    dob_.Q_b0 = aTs / (2.0 + aTs);
    dob_.Q_a0 = (aTs - 2.0) / (aTs + 2.0);

    dob_.PnQ_a0 = dob_.Q_a0;
    dob_.PnQ_b0 = (2*a*dob_.tau_uav + aTs) / (2.0 + aTs);
    dob_.PnQ_b1 = (aTs - 2*a*dob_.tau_uav) / (aTs + 2.0);
}

void PositionControl::uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose_t uav_pose;
    uav_pose.t = msg->header.stamp.toSec();
    uav_pose.p = point2vec(msg->pose.position);

    const Eigen::Vector4d q = quat2vec(msg->pose.orientation);
    uav_pose.R = q2R(q);
    uav_pose.head = computeHead(uav_pose.R.col(0));

    uav_pose_dq_.push_back(uav_pose);
    if (uav_pose_dq_.size() > MAX_SIZE) {
            uav_pose_dq_.pop_front();
    }

    if (uav_pose_dq_.empty()) {
        std::cerr << "Deque is empty. Cannot find closest timestamp." << std::endl;
        return;
    }

    size_t idx_min = 0; 
    double min_diff = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < uav_pose_dq_.size(); i++) {
        double diff = std::abs(uav_pose_dq_[i].t - gv_.t);
        if (diff < min_diff) {
            min_diff = diff;
            idx_min  = i;
        }
    }

    const auto &closest_pose = uav_pose_dq_[idx_min];

    // if (uav_pose_dq_.size() == MAX_SIZE) {
    //     std::cout << "Closest index: " << idx_min << std::endl;
    //     std::cout << "delay = " <<  closest_pose.t - uav_pose_dq_[MAX_SIZE-1].t << std::endl;
    // }

    uav_.t = closest_pose.t;
    uav_.p = closest_pose.p;
    uav_.R = closest_pose.R;
    uav_.head = closest_pose.head;

    // uav_.t = ros::Time::now().toSec();

    // uav_.p = point2vec(msg->pose.position);

    // const Eigen::Vector4d q = quat2vec(msg->pose.orientation);
    // const Eigen::Matrix3d R = q2R(q);
    // uav_.head = computeHead(R.col(0));
}

void PositionControl::uavTwistCb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uav_twist_t uav_twist;
    uav_twist.t = msg->header.stamp.toSec();
    uav_twist.v = vec2vec(msg->twist.linear);

    uav_twist_dq_.push_back(uav_twist);
    if (uav_twist_dq_.size() > MAX_SIZE) {
            uav_twist_dq_.pop_front();
    }

    if (uav_twist_dq_.empty()) {
        std::cerr << "Deque is empty. Cannot find closest timestamp." << std::endl;
        return;
    }

    size_t idx_min = 0; 
    double min_diff = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < uav_twist_dq_.size(); i++) {
        double diff = std::abs(uav_twist_dq_[i].t - gv_.t);
        if (diff < min_diff) {
            min_diff = diff;
            idx_min  = i;
        }
    }

    const auto &closest_pose = uav_twist_dq_[idx_min];

    // if (uav_twist_dq_.size() == MAX_SIZE) {
    //     std::cout << "Closest index: " << idx_min << std::endl;
    //     std::cout << "vel delay = " <<  closest_pose.t - uav_twist_dq_[MAX_SIZE-1].t << std::endl;
    // }

    uav_.v_prev = uav_.v;
    uav_.v = closest_pose.v;
    uav_.v = aims_fly::lpfv3(uav_.v, uav_.v_lpf_alpha, uav_.v_prev);

    // uav_.v_prev = uav_.v;
    // uav_.v = vec2vec(msg->twist.linear);
    // uav_.v = aims_fly::lpfv3(uav_.v, uav_.v_lpf_alpha, uav_.v_prev);
}

void PositionControl::uavImuCb(const sensor_msgs::Imu::ConstPtr &msg)
{
    const Eigen::Vector4d q = quat2vec(msg->orientation);
    const Eigen::Matrix3d R = q2R(q);
    uav_.b3 = R.col(2);

    const Eigen::Vector3d a_prev = uav_.a;
    uav_.a = R * vec2vec(msg->linear_acceleration);
    uav_.a = aims_fly::lpfv3(uav_.a, uav_.a_lpf_alpha, a_prev);

    // Acceleration Control (fast loop)
    accCtrl();
}

void PositionControl::uavFmCb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_.fm = msg->mode;
}

void PositionControl::hoverThrustCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    const bool valid = msg->pose.position.z;

    if (valid) {
        uav_.thr_h = msg->pose.position.x;
        cmd_.thr_l = 0.12 + (uav_.thr_h - 0.12) * sat_.acc_ratio;
    }
    else {
        uav_.thr_h = _thr_h_const;
    }
}

void PositionControl::fsmCb(const std_msgs::Int32::ConstPtr &msg)
{
    uav_.fsm_time = ros::Time::now().toSec();
    uav_.fsm = msg->data;
}


void PositionControl::gvEstiCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    const double t_prev = gv_.t;
    // gv_.t = ros::Time::now().toSec();
    gv_.t = msg->header.stamp.toSec();

    double dt = gv_.t - t_prev;
    if (dt <= aims_fly::N_EPS) {
        dt = sat_.dt;
        ROS_DEBUG("dt is too small");
    } else if (dt > 1.) {
        dt = 0.;
        ROS_DEBUG("dt is too large");
    }

    const Eigen::Vector4d q = quat2vec(msg->pose.pose.orientation);
    const Eigen::Matrix3d R = q2R(q);
    const double head_prev = gv_.head;
    gv_.head = computeHead(R.col(0));
    gv_.head = aims_fly::lpf(gv_.head, gv_.h_lpf_alpha, head_prev);
    if (flag_.fixed_yaw) {
        if (uav_.fsm == 4) {
            gv_.head = gv_.fixed_head;
        } else {
            gv_.fixed_head = gv_.head;
        }
    }
    
    // ROS_INFO("UAV Head: [%.2f] deg", aims_fly::rad2deg(uav_.head));
    // ROS_INFO("GV Head:  [%.2f] deg", aims_fly::rad2deg(gv_.head));

    //           lon              lat
    gv_.R << cos(gv_.head), -sin(gv_.head),
             sin(gv_.head),  cos(gv_.head);

    gv_.p = point2vec(msg->pose.pose.position);
    if (flag_.sitl) {
        gv_.p(2) = gv_.p(2) + 0.05;

        gv_.p(0) = gv_.p(0) + sat_.land_pnt_lon * cos(gv_.head);
        gv_.p(1) = gv_.p(1) + sat_.land_pnt_lon * sin(gv_.head); 
        gv_.p(2) = gv_.p(2) + sat_.land_pnt_alt;

        // Gaussian Noise (sigma)
        const double sigma = 0.05;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> dist(0.0, sigma);

        Eigen::Vector3d noise;
        noise << dist(gen), dist(gen), dist(gen);

        // std::cout << "UAV: "   << uav_.p(2) << std::endl;
        // std::cout << "GV: "    << gv_.p(2) << std::endl;
        // std::cout << "Noise: " << noise(2) << std::endl;

        gv_.p = gv_.p + noise;


    } else {
        gv_.p(0) = gv_.p(0) + sat_.land_pnt_lon * cos(gv_.head);
        gv_.p(1) = gv_.p(1) + sat_.land_pnt_lon * sin(gv_.head); 
        gv_.p(2) = gv_.p(2) + sat_.land_pnt_alt;
    }

    
    const Eigen::Vector3d v_prev = gv_.v;
    if (flag_.sitl) {
        gv_.v = R * vec2vec(msg->twist.twist.linear);

        // Gaussian Noise (sigma)
        const double sigma = 0.30;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> dist(0.0, sigma);

        Eigen::Vector3d noise;
        noise << dist(gen), dist(gen), dist(gen);

        // std::cout << "Original Data: " << gv_.v.transpose() << std::endl;
        
        gv_.v = gv_.v + noise;
        
        // std::cout << "Data w/ Noise: " << gv_.v.transpose() << std::endl;

    } else {
        gv_.v = vec2vec(msg->twist.twist.linear);
    }
    
    gv_.v = aims_fly::lpfv3(gv_.v, gv_.v_lpf_alpha, v_prev);
    // gv_.v(2) = 0.;

    

    // Position Control for tracking and landing control
    posCtrl(gv_.p, gv_.v, uav_.p, uav_.v);
    velCtrl(cmd_.v, uav_.v, uav_.a, dt);
    if (flag_.log) {
        logging();
    }
}

void PositionControl::posCtrl(const Eigen::Vector3d &p_des, const Eigen::Vector3d &v_des, const Eigen::Vector3d &p, const Eigen::Vector3d &v)
{
    // Standoff tracking
    const Eigen::Vector2d standoff = gv_.R * Eigen::Vector2d(cmd_.standoff(0), cmd_.standoff(1));
    // (1) compute the error
    rel_.p = p - p_des;
    rel_.v = v - v_des;
    const Eigen::Vector3d e = p_des - p + Eigen::Vector3d(standoff(0), standoff(1), cmd_.standoff(2));
    
    // (2) compute the control law
    Eigen::Vector3d u = P_.P * e + P_.D * (-v);
    if ((flag_.ff) && (uav_.fsm != 4)) {
        u = u + v_des;
    }

    // (3) saturation
    Eigen::Vector2d u_xy(u(0), u(1));
    if (u_xy.norm() > sat_.vel_xy) {
        u_xy = u_xy * (sat_.vel_xy / u_xy.norm());
    }

    if (abs(u(2)) > sat_.vel_z) {
        u(2) = u(2) * (sat_.vel_z / abs(u(2)));
    }

    // (4) fill in the command
    cmd_.v(0) = u_xy(0);
    cmd_.v(1) = u_xy(1);
    cmd_.v(2) = u(2);
}

void PositionControl::velCtrl(const Eigen::Vector3d &v_des, const Eigen::Vector3d &v, const Eigen::Vector3d &a, const double dt)
{
    // (1) compute the error
    const Eigen::Vector3d e = v_des - v;

    // (2) compute the control law
    Eigen::Vector3d u_c = V_.P * e + cmd_.int_v + V_.D * (-a) + g_;
    Eigen::Vector3d u;
    dob_.u_c = u_c - g_;

    if (flag_.dob) {
        estimateDst();
        u = u_c - dob_.w;
        // std::cout << "u_c: " << u_c.transpose() << std::endl;
        // std::cout << "u  : " << u.transpose()   << std::endl;
        dob_.u_dob_prev = dob_.u_dob;
        dob_.u_dob = u - g_;
    } else {
        u = u_c;
    }

    const Eigen::Vector3d cmd_a_prev = cmd_.a;

    if (uav_.fsm == 4) {
        // Proj
        const Eigen::Vector3d a_lat = getLatVec(u);

        // VF
        cmd_.a = calVFAccLaw();
        if (flag_.debug) {  
            std::cout << "a: " << u.transpose() << std::endl;
            std::cout << "a_lat: " << a_lat.transpose() << std::endl;
            std::cout << "a_vf: " << cmd_.a.transpose() << std::endl;
        }
        cmd_.a = cmd_.a + a_lat;
    }  
    else {
        cmd_.a = u;
    }

    // (3) acc2thrAtti
    cmd_.a = aims_fly::lpfv3(cmd_.a, cmd_.a_cmd_alpha, cmd_a_prev);
    acc2thrAtti(cmd_.a, uav_.b3);

    // (4) integral clamping
    if (uav_.fm != fm_.offboard) {
        cmd_.int_v << 0., 0., 0.;
    }
    else {
        if (uav_.fsm == 4) {
            cmd_.int_v << 0., 0., 0.;
            
        }
        else {
            cmd_.int_v += (V_.I * e) * dt;

            // anti-windup clamping
            for (int i=0; i<2; ++i) {
                cmd_.int_v(i) = clamp(cmd_.int_v(i), -sat_.int_vxy, sat_.int_vxy);
            }
            cmd_.int_v(2) = clamp(cmd_.int_v(2), -sat_.int_vz, sat_.int_vz);
        }
    }
}

void PositionControl::acc2thrAtti(const Eigen::Vector3d &a_des, const Eigen::Vector3d &b3)
{
    // (1) compute the desired frame
    const Eigen::Vector3d d3 = (a_des).normalized();

    // (2) saturation for a desired attitude
    const Eigen::Vector3d d3_sat = limitTilt(d3);

    cmd_.thr_c = computeCollectiveThrust(a_des, b3);
    cmd_.q = computeDesiredAttitude(gv_.head, uav_.head, d3_sat);
}

void PositionControl::accCtrl()
{
    // Compute the desired collective thrust @ 200 Hz
    cmd_.thr_c = computeCollectiveThrust(cmd_.a, uav_.b3);
    if (flag_.debug) {
        std::cout << "Thr cmd: " << cmd_.thr_c << std::endl;
    }

    if ((uav_.fsm == 0) || (uav_.fsm == 1) || (uav_.fsm == 5)) {
        return;
    }

    if (flag_.landed) {
        Eigen::Matrix3d R;
        R << cos(gv_.head), -sin(gv_.head), 0.,
             sin(gv_.head),  cos(gv_.head), 0.,
                        0.,             0., 1.;
        cmd_.q = R2q(R);
        cmd_.thr_c = cmd_.thr_l;

        if (abs(ros::Time::now().toSec() - sat_.t) > sat_.t_end) {
            cmd_.thr_c = 0.;
        }
    }
    publish(cmd_.q, cmd_.thr_c);
}

void PositionControl::estimateDst()
{
    if (uav_.fm == fm_.offboard) {

        dob_.w2 = dob_.PnQ_b0*uav_.v + dob_.PnQ_b1*uav_.v_prev - dob_.PnQ_a0*dob_.w2;

        dob_.w1 = (dob_.Q_b0*(dob_.u_c-dob_.w2+dob_.u_dob)-dob_.Q_a0*dob_.w1) / (1.- dob_.Q_b0);
        // dob_.w1 = 0.006244*(dob_.u_dob + dob_.u_dob_prev) + 0.9875*dob_.w1;
        
        dob_.w = dob_.w2 - dob_.w1;
    }

    // std::cout << "w: " << dob_.w.transpose() << std::endl;
}

double PositionControl::computeHead(const Eigen::Vector3d &b1)
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

Eigen::Vector3d PositionControl::getLatVec(const Eigen::Vector3d &u)
{   
    // This function extracts lateral acc. for concating with vector field acceleration law
    // [x,y]^T = [c,-s; s,c] [lon,lat]^T  
    // lat = [-s,c]^T [x,y]
    // Thereforfe, [x,y]^T = [c,-s; s,c] [0,lat]^T = [-s,c]*lat

    Eigen::Vector2d uxy(u(0), u(1));
    uxy = (uxy.dot(gv_.R.col(1)))*gv_.R.col(1);
    return Eigen::Vector3d(uxy(0), uxy(1), 0.) + g_;
}

Eigen::Vector3d PositionControl::calVFAccLaw()
{
    const Eigen::Vector2d rel_pos_xy(rel_.p(0), rel_.p(1));                                                    // rel_.p = (xr, yr, zr) = p_uav - p_gv
    const double rel_pos_xy_lon = rel_pos_xy.dot(gv_.R.col(0));                                                // x := projection into longitudial axis (GV head)
    
    const double rel_r = sqrt(rel_pos_xy_lon*rel_pos_xy_lon + rel_.p(2)*rel_.p(2));                            // r > 0
    const double rel_phi_err = atan2(rel_pos_xy_lon, rel_.p(2)) - vf_.phi_des;                                 // phi_tilde = phi - phi_des,  phi=atan(x/z)
    
    const Eigen::Vector2d rel_vel_xy(rel_.v(0), rel_.v(1));                                                    // rel_.v = (vr_x, vr_y, vr_z) = v_uav - v_gv
    const double rel_vel_xy_lon = rel_vel_xy.dot(gv_.R.col(0));                                                // x_dot

    const double rel_r_dot = (rel_pos_xy_lon * rel_vel_xy_lon + rel_.p(2) * rel_.v(2)) / rel_r;  // r_dot
    const double rel_phi_err_dot = (rel_.p(2) * rel_vel_xy_lon - rel_pos_xy_lon * rel_.v(2)) / pow(rel_r, 2);  // phi_tilde_dot = phi_dot

    rel_.eta << rel_r, rel_phi_err;                                                                            // eta = (r, phi_tilde)
    rel_.eta_dot << rel_r_dot, rel_r * rel_phi_err_dot;                                                        // eta_dot = r_dot*e_r + (r*phi_tilde_dot)*e_phi

    // vector field (h = c*e_r + s*e_phi)
    double c, s;
    if (vf_.hyper) {
        c = -vf_.k1 * sech(rel_phi_err / vf_.c1) * pow(rel_r, 1./vf_.n2);
        s = -vf_.k2 * tanh(rel_phi_err / vf_.c2) * pow(rel_r, 1./vf_.n2);

    } else {
        c = -vf_.k1 * exp(-pow((rel_phi_err / vf_.c1), vf_.n1)) * pow(rel_r, 1./vf_.n2);
        s = -vf_.k2 * tanh(rel_phi_err * vf_.c2) * rel_r;
    }

    vf_.h << c, s;

    if (vf_.moving) {
        Eigen::Vector2d gv_v_xy(gv_.v(0), gv_.v(1));
        const double gv_v_lon = gv_v_xy.dot(gv_.R.col(0));
        const double gv_phi = atan2(gv_v_lon, gv_.v(2));

        Eigen::Matrix2d _R;
        _R << sin(gv_phi), cos(gv_phi),
              cos(gv_phi), -sin(gv_phi);
        
        const Eigen::Vector2d h_ff = _R * Eigen::Vector2d(gv_v_lon, gv_.v(2));

        vf_.h = vf_.h + h_ff;
    }

    Eigen::Matrix2d Ja_h;
    if (vf_.hyper) {
        Ja_h(0,0) = (-vf_.k1 / vf_.n2) * sech(rel_phi_err / vf_.c1) * pow(rel_r, (1./vf_.n2) - 1);
        Ja_h(0,1) = (vf_.k1 / vf_.c1) * pow(rel_r, 1./vf_.n2) * sech(rel_phi_err / vf_.c1) * tanh(rel_phi_err / vf_.c1);
        Ja_h(1,0) = (-vf_.k2 / vf_.n2) * tanh(rel_phi_err / vf_.c2) * pow(rel_r, (1./vf_.n2) - 1);
        Ja_h(1,1) = -vf_.k2 * vf_.c2 * rel_r * aims_fly::tanh_derivative(rel_phi_err * vf_.c2);

    } else {
        Ja_h(0,0) = (-vf_.k1 / vf_.n2) * exp(-pow((rel_phi_err / vf_.c1), vf_.n1)) * pow(rel_r, (1./vf_.n2) - 1);
        Ja_h(0,1) = (vf_.k1 * vf_.n1 / vf_.c1) * pow(rel_r, 1./vf_.n2) * exp(-pow((rel_phi_err / vf_.c1), vf_.n1)) * pow((rel_phi_err / vf_.c1), vf_.n1 - 1);
        Ja_h(1,0) = -vf_.k2 * tanh(rel_phi_err * vf_.c2);
        Ja_h(1,1) = (-vf_.k2 / vf_.c2) * pow(rel_r, 1./vf_.n2) * aims_fly::tanh_derivative(rel_phi_err / vf_.c2);
    }

    const Eigen::Vector2d h_dot = Ja_h * rel_.eta_dot;

    Eigen::Vector2d a_law = -vf_.gamma * (rel_.eta_dot - vf_.h) + h_dot;  // (er, e\phi)
    
    // r-phi <-> x_lon-z
    const double phi = rel_phi_err + vf_.phi_des;
    Eigen::Matrix2d R_inv;
    R_inv << sin(phi), cos(phi),
             cos(phi), -sin(phi);
    
    a_law = R_inv * a_law;                                                                      // (xy_lon, z)

    vf_.a_law = Eigen::Vector3d(a_law(0) * cos(gv_.head), a_law(0) * sin(gv_.head), a_law(1));  // (x, y, z)

    // bool _debug = true;
    if (flag_.debug) {
        std::cout << "| eta:          " << rel_.eta.transpose() << std::endl;
        std::cout << "| eta_dot:      " << rel_.eta_dot.transpose() << std::endl;
        std::cout << "| h:            " << vf_.h.transpose() << std::endl;
        std::cout << "| h_dot:        " << h_dot.transpose() << std::endl;
        std::cout << "| a_law_lon:    " << a_law.transpose() << std::endl;
        std::cout << "| a_law:        " << vf_.a_law.transpose() << std::endl;
    }

    return vf_.a_law;
}

double PositionControl::computeCollectiveThrust(const Eigen::Vector3d &a_cmd, const Eigen::Vector3d &b3)
{
    // convert acceleration to normalized collective thrust
    const double thr_c = a_cmd.dot(b3) * uav_.thr_h / g_.norm();

    const double thr_c_sat = clamp(thr_c, sat_.thr(0), sat_.thr(1));

    return thr_c_sat;
}

Eigen::Vector3d PositionControl::limitTilt(const Eigen::Vector3d &d3)
{
    Eigen::Vector3d d3_sat = d3;
    const double tilt = acos(d3.dot(w3_));
    
    if (tilt >= sat_.tilt) {
        const double lambda = sin(sat_.tilt) / sqrt(pow(d3(0),2) + pow(d3(1),2));
        
        d3_sat(0) *= lambda;
        d3_sat(1) *= lambda;
        d3_sat(2) = cos(sat_.tilt); 
    }

    return d3_sat;
}

Eigen::Vector4d PositionControl::computeDesiredAttitude(const double head_des, const double head, const Eigen::Vector3d &d3)
{
    // Compute the Rot_W_D
    const double lambda = sqrt(pow(d3(2),2) / (pow(d3(2),2) + pow(cos(head_des) * d3(0) + sin(head_des) * d3(1), 2)));

    // Check for NaN
    if (std::isnan(head_des) || std::isnan(head) || std::isnan(lambda))
    {
        throw std::runtime_error(
            "Error in computeDesiredAttitude: head_des, head, or lambda is NaN."
        );
    }
    
    const Eigen::Vector3d d1(lambda*cos(head_des), lambda*sin(head_des), -lambda*(cos(head_des) * d3(0) + sin(head_des) * d3(1))/d3(2));
    
    const Eigen::Vector3d d2 = d3.cross(d1);
    
    Eigen::Matrix3d R;
    R << d1, d2, d3;

    return R2q(R);
}


double PositionControl::clamp(const double x, const double x_min, const double x_max)
{
    return std::min(std::max(x, x_min), x_max);
}

void PositionControl::publish(const Eigen::Vector4d &qW_D, const double thr_c)
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.type_mask = 1+2+4;
    msg.orientation.w = qW_D(0);
    msg.orientation.x = qW_D(1);
    msg.orientation.y = qW_D(2);
    msg.orientation.z = qW_D(3);
    msg.thrust = thr_c;
    pub_attitarget_.publish(msg);
}

void PositionControl::monitor(const ros::TimerEvent &event)
{
    // prevent unexpected action when nodes suddenly died 
    const double t = ros::Time::now().toSec();
    if (uav_.fm == fm_.offboard) {
        if ((abs(t-uav_.t) > sat_.T) || (abs(t-gv_.t) > sat_.T) || (abs(t-uav_.fsm_time) > sat_.T)) {
            changeFlightMode(fm_.hold);
            initCmd();
            ROS_INFO("No measurement in time");
        }

        ROS_INFO("To critical altitude: %.2f", rel_.p(2) - sat_.crt_alt + sat_.land_pnt_alt);
        if ((rel_.p(2) < (sat_.crt_alt - sat_.land_pnt_alt)) && (flag_.landed == false)) {
            flag_.landed = true;
            sat_.t = ros::Time::now().toSec();
            sat_.t_end = sqrt(2. * sat_.crt_alt * cmd_.thr_l);
            // landed();
        }
    }
}

void PositionControl::changeFlightMode(const std::string &mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;

    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("Mode changed to %s", mode.c_str());
    } else {
        ROS_WARN("Failed to change mode to %s", mode.c_str());
    }
}

void PositionControl::landed()
{
    // Wait for the service to become available
    if (!set_mode_client_.waitForExistence(ros::Duration(0.1))) {
        ROS_ERROR("SetMode service not available!");
        return;
    }

    // Create a service request
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 92;  // MAV_MODE_AUTO_DISARMED
    srv.request.custom_mode = "";  // Leave blank to use base_mode

    // Call the service
    if (set_mode_client_.call(srv)) {
        if (srv.response.mode_sent) {
            ROS_INFO("Successfully changed mode to AUTO DISARMED");
        } else {
            ROS_WARN("Mode change request sent, but not confirmed.");
        }
    } else {
        ROS_ERROR("Failed to call set_mode service");
    }
}

std::string PositionControl::getTimestampFilename() 
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm* now_tm = std::localtime(&now_time);

    std::ostringstream oss;
    oss << "log_"
        << std::put_time(now_tm, "%Y%m%d_%H%M%S")
        << ".csv";

    return oss.str();
}

void PositionControl::logging()
{
    if (log_.is_open()) {
        log_ << std::fixed << std::setprecision(5);
        log_ << ros::Time::now().toSec() << "," << uav_.p(0) << "," << uav_.p(1) << "," << uav_.p(2) << "," << uav_.v(0) << "," << uav_.v(1) << "," << uav_.v(2) << "," << uav_.a(0) << "," << uav_.a(1) << "," << uav_.a(2) << "," << uav_.head << "," << gv_.p(0) << "," << gv_.p(1) << "," << gv_.p(2) << "," << gv_.v(0) << "," << gv_.v(1) << "," << gv_.v(2) << "," << gv_.head << "," << cmd_.v(0) << "," << cmd_.v(1) << "," << cmd_.v(2) << "," << cmd_.int_v(0) << "," << cmd_.int_v(1) << "," << cmd_.int_v(2) << "," << cmd_.a(0) << "," << cmd_.a(1) << "," << cmd_.a(2) << "," << uav_.b3(0) << "," << uav_.b3(1) << "," << uav_.b3(2) << "," << vf_.h(0) << "," << vf_.h(1)  << "\n";
    }
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
