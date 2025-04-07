#include "aims_als/tag_esti.hpp"

EstimationTag::EstimationTag(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const std::string &topicname):
nh_(nh),
nh_private_(nh_private),
topicname_(topicname)
{
    nh_private_.param<double>("sample_time", _dt, 1./10.);
    nh_private_.param<double>("initial_state_uncertainty", _P0, 500.);
    nh_private_.param<double>("model_acc_uncertainty", _acc_cov, 10.);
    nh_private_.param<double>("meas_uncertainty", _meas_cov, 0.014);
    nh_private_.param<double>("gv_z_alpha", gv_.alpha, 1.0);


    sub_pose_tag_tf_   = nh_.subscribe("/aims/" + topicname_, 1, &EstimationTag::measureCb, this, ros::TransportHints().tcpNoDelay());
    // pub_odom_tag_esti_ = nh_.advertise<nav_msgs::Odometry>("/aims/" + topicname_ + "_esti", 5);
    pub_odom_tag_esti_ = nh_.advertise<nav_msgs::Odometry>("/aims/gv_esti", 1);

    
    F_t F; 
    F << 1, 0, _dt,  0, 0.5*(_dt*_dt),             0,  // constant acceleration model
         0, 1,  0, _dt,             0, 0.5*(_dt*_dt),
         0, 0,  1,   0,           _dt,             0,
         0, 0,  0,   1,             0,           _dt,
         0, 0,  0,   0,             1,             0,
         0, 0,  0,   0,             0,             1;
    // F << 1, 0, _dt,  0,                                // constant velocity model
    //      0, 1,  0, _dt,            
    //      0, 0,  1,   0,           
    //      0, 0,  0,   1;                 


    G_t G; 
    G << 0,                                            // constant acceleration model
         0,
         0, 
         0, 
         0, 
         0;
    // G << 0,                                            // constant velocity model
    //      0,
    //      0, 
    //      0; 


    H_t H;
    H << 1, 0, 0, 0, 0, 0,                             // constant acceleration model
         0, 1, 0, 0, 0, 0; 
    // H << 1, 0, 0, 0, 0, 0,                                // constant acceleration model w/ velocity measurements
    //      0, 1, 0, 0, 0, 0,
    //      0, 0, 1, 0, 0, 0,
    //      0, 0, 0, 1, 0, 0;
    // H << 1, 0, 0, 0,                                      // constant velocity model
    //      0, 1, 0, 0; 
    // H << 1, 0, 0, 0,                                      // constant velocity model w/ velocity measurements
    //      0, 1, 0, 0,
    //      0, 0, 1, 0,
    //      0, 0, 0, 1; 

    Q_t Q = setQd();

    R_t R = R_t::Identity() * _meas_cov;                  // constant acceleration/velocity model
    // R_t R = R_t::Identity() * _meas_cov;                  // constant acceleration/velocity model w/ velocity measurements
    // double vel_meas_cov = 0.08;
    // R(2,2) = vel_meas_cov;
    // R(3,3) = vel_meas_cov;

    lkf_.setMatrices(F, G, H);
    lkf_.setCovMatrices(Q, R);

    x_hat_.x = x_t::Zero();
    x_hat_.P = P_t::Identity();
    x_hat_ = lkf_.init(x_hat_, _P0);

    u_ = u_t::Zero();
    z_ = z_t::Zero();

    gv_.z_esti = 0.;
    gv_.z_meas = 0.;
    gv_.q << 1., 0., 0., 0.; 

    ROS_INFO("Ground Vehicle of %s Estimator is initialized...!", topicname_.c_str());
}

void EstimationTag::init()
{
    x_hat_.x = x_t::Zero();
    x_hat_.P = P_t::Identity();
    x_hat_ = lkf_.init(x_hat_, _P0);

    u_ = u_t::Zero();
    z_ = z_t::Zero();

    t_prev_ = 0.;
}

Q_t EstimationTag::setQd()
{
    Q_t Qd = Q_t::Zero();

    Qd(0,0) = pow(_dt,4) / 4.;                            // constant acceleration model
    Qd(0,2) = pow(_dt,3) / 2.;
    Qd(0,4) = pow(_dt,2) / 2.;

    Qd(1,1) = pow(_dt,4) / 4.;
    Qd(1,3) = pow(_dt,3) / 2.;
    Qd(1,5) = pow(_dt,2) / 2.;

    Qd(2,0) = pow(_dt,3) / 2.;
    Qd(2,2) = pow(_dt,2);
    Qd(2,4) = _dt;

    Qd(3,1) = pow(_dt,3) / 2.;
    Qd(3,3) = pow(_dt,2);
    Qd(3,5) = _dt;
	
    Qd(4,0) = pow(_dt,2) / 2.;
    Qd(4,2) = _dt;
    Qd(4,4) = 1.;

    Qd(5,1) = pow(_dt,2) /2.;
    Qd(5,3) = _dt;
    Qd(5,5) = 1.;

    // Qd(0,0) = pow(_dt,4) / 4.;                            // constant velocity model
    // Qd(0,2) = pow(_dt,3) / 2.;

    // Qd(1,1) = pow(_dt,4) / 4.;
    // Qd(1,3) = pow(_dt,3) / 2.;

    // Qd(2,0) = pow(_dt,3) / 2.;
    // Qd(2,2) = pow(_dt,2);

    // Qd(3,1) = pow(_dt,3) / 2.;
    // Qd(3,3) = pow(_dt,2);

    Qd *= _acc_cov;
    return Qd;
}

void EstimationTag::publish(const esti_t &x_hat)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header = header_;
    odom_msg.pose.pose.position.x = x_hat.x(0);
    odom_msg.pose.pose.position.y = x_hat.x(1);
    odom_msg.twist.twist.linear.x = x_hat.x(2);
    odom_msg.twist.twist.linear.y = x_hat.x(3);

    odom_msg.pose.pose.position.z = gv_.z_esti;
    odom_msg.pose.pose.orientation.w = gv_.q(0);
    odom_msg.pose.pose.orientation.x = gv_.q(1);
    odom_msg.pose.pose.orientation.y = gv_.q(2);
    odom_msg.pose.pose.orientation.z = gv_.q(3);

    pub_odom_tag_esti_.publish(odom_msg);
}

void EstimationTag::measureCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    header_ = msg->header;
    gv_.z_meas = msg->pose.position.z;
    gv_.z_esti = gv_.z_esti + gv_.alpha * (gv_.z_meas - gv_.z_esti);
    gv_.q = quat2vec(msg->pose.orientation);

    const double dt = ros::Time::now().toSec() - t_prev_;
    if (dt >= _dt*10) {
        init();
    }
    t_prev_ = ros::Time::now().toSec();

    // constant acceleration/velocity model w/o velocity measurements
    z_ << msg->pose.position.x, msg->pose.position.y;

    // // constant acceleration/velocity model w/ velocity measurements
    // z_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->twist.twist.linear.x, msg->twist.twist.linear.y;

    // // // Treating Outliers: projection into driving direction
    // const Eigen::Matrix3d R33 = q2R(gv_.q);
    // Eigen::Matrix2d R22 = R33.block<2,2>(0,0);
    // Eigen::Matrix2d I;
    // I << 1., 0.,
    //      0., _lambda;                             // rows and columns start at (0, 0)
    // const Eigen::Vector2d V_proj = R22.transpose() * I * R22 * Eigen::Vector2d(z_(2), z_(3));

    // z_(2) = V_proj(0);
    // z_(3) = V_proj(1);

    // 'try-catch' to prevent program crashes
    // in case of numerical instabilities (e.g. matrix inversion)
    try {   
      x_hat_ = lkf_.predict(x_hat_, u_);      
      x_hat_ = lkf_.correct(x_hat_, z_);
      publish(x_hat_);

    } catch (const std::exception& e) {
      ROS_ERROR("LKF failed: %s", e.what());
    }

    // ROS_INFO("Meas X: %.2f", z_(0));
    // ROS_INFO("Esti X: %.2f", x_hat_.x(0));
    // ROS_INFO("Esti Pxx: %.2f", x_hat_.P(0,0));
    // ROS_INFO("Esti Vx: %.2f", x_hat_.x(2));
    // ROS_INFO("Esti Ax: %.2f", x_hat_.x(4));
}

    // ROS_INFO("Esti X: %.2f", x_hat_.x(0));
    // ROS_INFO("Esti Pxx: %.2f", x_hat_.P(0,0));
    // ROS_INFO("Esti Vx: %.2f", x_hat_.x(2));
    // ROS_INFO("Esti Ax: %.2f", x_hat_.x(4));

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_esti_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    /* Two-Apriltags */ 
    // std::string b_topicname = "b_tag_tf"; 
    // std::string s_topicname = "s_tag_tf";

    // auto b_gv_esti_obj = std::make_unique<EstimationTag>(nh, nh_private, b_topicname);
    // auto s_gv_esti_obj = std::make_unique<EstimationTag>(nh, nh_private, s_topicname);
    
    // // Use an AsyncSpinner to handle callbacks from both objects in parallel
    // ros::AsyncSpinner spinner(2);  // Use 2 threads for 2 objects
    // spinner.start();

    // // To keep the node alive until it's shut down
    // ros::waitForShutdown();

    /* Fractal Marker */
    std::string topicname = "transformed_marker";
    auto gv_esti_obj = std::make_unique<EstimationTag>(nh, nh_private, topicname);

    gv_esti_obj->run();
    
    return 0;
}
