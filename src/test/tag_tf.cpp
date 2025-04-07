#include "aims_als/tag_tf.hpp"


TransformationTag::TransformationTag(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private):
nh_(nh),
nh_private_(nh_private)
{
    nh_private_.param<std::string>("world_frame", world_frame_, "map");
    nh_private_.param<std::string>("uav_frame", uav_frame_, "base_link");
    nh_private_.param<std::string>("camera_frame", cam_frame_, "camera_link");
    nh_private_.param<double>("node_rate", _dt, 20.);
    nh_private_.param<double>("lpf_alpha", _alpha, 1.);  // w/o lpf == alpha=1
    _dt = 1./_dt;  // freq to sec

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    sub_pose_b_tag_ = nh_.subscribe("/aims/b_tag_detections", 1, &TransformationTag::btagPoseCb, this, ros::TransportHints().tcpNoDelay());
    sub_pose_s_tag_ = nh_.subscribe("/aims/s_tag_detections", 1, &TransformationTag::stagPoseCb, this, ros::TransportHints().tcpNoDelay());
    sub_odom_uav_   = nh_.subscribe("/mavros/local_position/odom", 1, &TransformationTag::uavOdomCb, this, ros::TransportHints().tcpNoDelay());
    
    pub_odom_b_tag_ = nh_.advertise<nav_msgs::Odometry>("/aims/b_tag_tf", 1);
    pub_odom_s_tag_ = nh_.advertise<nav_msgs::Odometry>("/aims/s_tag_tf", 1);

    b_tag_prev_f_.first_iter = true;
    b_tag_prev_f_.t = ros::Time::now().toSec();
    b_tag_prev_f_.x = 0.;
    b_tag_prev_f_.y = 0.;
}

void TransformationTag::btagPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    try {
        const Eigen::Vector3d pC_GV = point2vec(msg->pose.position);
		const Eigen::Vector3d pW_GV = rotVecWithT(Tw_c_, pC_GV);

		const Eigen::Vector4d qC_T = quat2vec(msg->pose.orientation);
		const Eigen::Vector4d qW_C = R2q(Rw_c_);
		const Eigen::Vector4d qW_T = quatProd(qW_C, qC_T);

        const Eigen::Vector3d Ew_T = q2E(qW_T);
        const Eigen::Vector4d qW_h = E2q(Eigen::Vector3d(0., 0., Ew_T(2)));  // heading

        // compute velocity
        const double t_now = ros::Time::now().toSec();
        double dt = _dt;
        if (b_tag_prev_f_.first_iter) {
            b_tag_prev_f_.first_iter = false;
            b_tag_prev_f_.x = pW_GV(0);
            b_tag_prev_f_.y = pW_GV(1);
        } else {
            dt = t_now - b_tag_prev_f_.t;
            if (abs(dt) <= std::numeric_limits<double>::min()) {
                dt = _dt;
            }
            dt = std::min(dt, _dt);
        }

        const auto vx = (pW_GV(0) - b_tag_prev_f_.x) / dt;
        const auto vy = (pW_GV(1) - b_tag_prev_f_.y) / dt;

        // publish
        nav_msgs::Odometry tf_odom;
        tf_odom.header.stamp = ros::Time::now();
        tf_odom.pose.pose.position.x = pW_GV(0);
        tf_odom.pose.pose.position.y = pW_GV(1);
		tf_odom.pose.pose.position.z = pW_GV(2);
        tf_odom.pose.pose.orientation.w = qW_h(0); 
        tf_odom.pose.pose.orientation.x = qW_h(1); 
        tf_odom.pose.pose.orientation.y = qW_h(2); 
        tf_odom.pose.pose.orientation.z = qW_h(3); 
        tf_odom.twist.twist.linear.x = vx;
        tf_odom.twist.twist.linear.y = vy;
        pub_odom_b_tag_.publish(tf_odom);

        b_tag_prev_f_.t = t_now;
        b_tag_prev_f_.x = pW_GV(0);
        b_tag_prev_f_.y = pW_GV(1);

        head_prev_f_ = Ew_T(2);




        // geometry_msgs::TransformStamped tf_stamped;
        // tf_stamped = tf_buffer_->lookupTransform(world_frame_, cam_frame_, ros::Time(0));

        // geometry_msgs::PoseStamped tf_pose_stamped;
        // tf2::doTransform(*msg, tf_pose_stamped, tf_stamped);

        // nav_msgs::Odometry tf_odom;
        // tf_odom.pose.pose = tf_pose_stamped.pose;
        // ROS_INFO("TF version = [%.3f, %.3f, %.3f]", tf_pose_stamped.pose.position.x, tf_pose_stamped.pose.position.y, tf_pose_stamped.pose.position.z);


        // extract heading (yaw)
        // auto q = quat2vec(tf_odom.pose.pose.orientation);
        // auto R = q2R(q);
        // auto E = q2E(q);
        // const Eigen::Vector2d eW_x_12(1., 0.);
        // const Eigen::Vector2d eB_x_12(R(0,0), R(1,0));
        // double head = acos(eB_x_12.dot(eW_x_12) / eB_x_12.norm());
        // if (R(1,0) < 0) { // 1&2 quadrant -> head>0
        //     head = -head; // 3&4 quadrant -> head<0
        // } 
        // const double head_f = lpf(head, _alpha, head_prev_f_);
        // const auto q_h = E2q(Eigen::Vector3d(0.,0.,head));         
        
        // // LPF for position
        // const double x_f = lpf(pW_GV(0), _alpha, b_tag_prev_f_.x);
        // const double y_f = lpf(pW_GV(1), _alpha, b_tag_prev_f_.y);

        // // compute velocity
        // const double t_now = ros::Time::now().toSec();
        // double dt = _dt;
        // if (b_tag_prev_f_.first_iter) {
        //     b_tag_prev_f_.first_iter = false;
        // } else {
        //     dt = t_now - b_tag_prev_f_.t;
        //     if (abs(dt) <= std::numeric_limits<double>::min()) {
        //         dt = _dt;
        //     }
        //     dt = std::min(dt, _dt);
        // }

        // const auto vx = (x_f - b_tag_prev_f_.x) / dt;
        // const auto vy = (y_f - b_tag_prev_f_.y) / dt;

        // // publish
        // tf_odom.pose.pose.position.x = pW_GV(0);
        // tf_odom.pose.pose.position.y = pW_GV(1);
		// tf_odom.pose.pose.position.z = pW_GV(2);
        // tf_odom.pose.pose.orientation.w = qW_T(0); //q_h(0);
        // tf_odom.pose.pose.orientation.x = qW_T(1); //q_h(1);
        // tf_odom.pose.pose.orientation.y = qW_T(2); //q_h(2);
        // tf_odom.pose.pose.orientation.z = qW_T(3); //q_h(3);
        // tf_odom.twist.twist.linear.x = vx;
        // tf_odom.twist.twist.linear.y = vy;
        // pub_odom_b_tag_.publish(tf_odom);

        // b_tag_prev_f_.t = t_now;
        // b_tag_prev_f_.x = x_f;
        // b_tag_prev_f_.y = y_f;

        // head_prev_f_ = head_f;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could not transform: %s", ex.what());
    }
}

void TransformationTag::stagPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    try {
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped = tf_buffer_->lookupTransform(world_frame_, cam_frame_, ros::Time(0));

        geometry_msgs::PoseStamped tf_pose_stamped;
        tf2::doTransform(*msg, tf_pose_stamped, tf_stamped);

        nav_msgs::Odometry tf_odom;
        tf_odom.pose.pose = tf_pose_stamped.pose;

        // extract heading (yaw)
        auto q = quat2vec(tf_odom.pose.pose.orientation);
        auto E = q2E(q);
        Eigen::Vector4d q_h = E2q(Eigen::Vector3d(0.,0.,E(2)));

        // compute velocity
        const double t_now = ros::Time::now().toSec();
        double dt = _dt;
        if (s_tag_prev_f_.first_iter) {
            s_tag_prev_f_.first_iter = false;
        } else {
            dt = t_now - s_tag_prev_f_.t;
            if (abs(dt) <= std::numeric_limits<double>::min()) {
                dt = _dt;
            }
            dt = std::min(dt, _dt);
        }

        const auto vx = (tf_odom.pose.pose.position.x - s_tag_prev_f_.x) / dt;
        const auto vy = (tf_odom.pose.pose.position.y - s_tag_prev_f_.y) / dt;

        // publish
        // tf_odom.pose.pose.position.x = tf_odom.pose.pose.position.x;
        // tf_odom.pose.pose.position.y = tf_odom.pose.pose.position.y;
        tf_odom.pose.pose.orientation.w = q_h(0);
        tf_odom.pose.pose.orientation.x = q_h(1);
        tf_odom.pose.pose.orientation.y = q_h(2);
        tf_odom.pose.pose.orientation.z = q_h(3);
        tf_odom.twist.twist.linear.x = vx;
        tf_odom.twist.twist.linear.y = vy;
        pub_odom_s_tag_.publish(tf_odom);

        s_tag_prev_f_.t = t_now;
        s_tag_prev_f_.x = tf_odom.pose.pose.position.x;
        s_tag_prev_f_.y = tf_odom.pose.pose.position.y;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could not transform: %s", ex.what());
    }
}

void TransformationTag::uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    /** @test 
     *  Real-world
     *  - {W}: ENU convention in ROS and NED convention in PX4 firmware
     *  - {B}: FLU convention in ROS
     */
    /* Real-world test:  */
    // geometry_msgs::TransformStamped tf_msg, tf_msg2;

    // tf_msg.header.stamp = ros::Time::now();
    // tf_msg.header.frame_id = world_frame_;
    // tf_msg.child_frame_id = uav_frame_;
    // tf_msg.transform.translation.x = msg->pose.pose.position.x;
	// tf_msg.transform.translation.y = msg->pose.pose.position.y;
	// tf_msg.transform.translation.z = msg->pose.pose.position.z;
    // tf_msg.transform.rotation = msg->pose.pose.orientation;
    
    // Eigen::Vector4d q;
    // q << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
    // const Eigen::Vector3d E = q2E(q);
    // const Eigen::Vector3d E_b2c(-M_PI, 0., E(2));
    // const Eigen::Vector4d q_b2c = E2q(E_b2c);
    // // ROS_INFO("Yaw: [%.2f]", E(2)*180./M_PI);

    // tf_msg2.header.stamp = tf_msg.header.stamp;
    // tf_msg2.header.frame_id = world_frame_;
    // tf_msg2.child_frame_id = cam_frame_;
    // tf_msg2.transform.translation.x = 0.;
	// tf_msg2.transform.translation.y = 0.;
	// tf_msg2.transform.translation.z = -0.15;
    // tf_msg2.transform.rotation.w = q_b2c(0);
    // tf_msg2.transform.rotation.x = q_b2c(1);
    // tf_msg2.transform.rotation.y = q_b2c(2);
    // tf_msg2.transform.rotation.z = q_b2c(3);
    
	// map2uav_br_.sendTransform(tf_msg);
    // map2cam_br_.sendTransform(tf_msg2);


    // pw_GV = Tw_c * pc_GV (raw data) 
    // Hence, construct the Tw_c
    Eigen::Vector4d qW_B;
    qW_B << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
    const Eigen::Vector3d Ew_b = q2E(qW_B);
    Rw_c_ = E2R(Eigen::Vector3d(PI, 0., Ew_b(2)));

    const Eigen::Vector3d tw_c = point2vec(msg->pose.pose.position) + Eigen::Vector3d(0., 0., -0.15);
    
    Tw_c_ = setMatrixT(Rw_c_, tw_c);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_tf_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    auto tag_tf_obj = std::make_unique<TransformationTag>(nh, nh_private);
    
    tag_tf_obj->run();

    return 0;
}
