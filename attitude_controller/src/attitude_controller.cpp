
#include "attitude_controller/attitude_controller.h"

using namespace Eigen;
using namespace std;
// Constructor

attitudeCtrl::attitudeCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
	: nh_(nh),
	  nh_private_(nh_private),
	  fail_detec_(false),
	  ctrl_enable_(true),
	  landing_commanded_(false),
	  feedthrough_enable_(false),
	  node_state(WAITING_FOR_HOME_POSE) 
{
  // 订阅 trajectory_publisher
  referenceSub_     = nh_.subscribe("reference/setpoint", 1, &attitudeCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  // flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &attitudeCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());

  // 订阅 Mavros
  mavstateSub_      = nh_.subscribe("mavros/state", 1, &attitudeCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_       = nh_.subscribe("mavros/local_position/pose", 1, &attitudeCtrl::mavposeCallback, this, ros::TransportHints().tcpNoDelay());
  // mavposeSub_       = nh_.subscribe("/mavros/vision_pose/pose", 1, &attitudeCtrl::mavposeCallback, this, ros::TransportHints().tcpNoDelay());
  mavtwistSub_      = nh_.subscribe("mavros/local_position/velocity_local", 1, &attitudeCtrl::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());

  ctrltriggerServ_  = nh_.advertiseService("tigger_rlcontroller", &attitudeCtrl::ctrltriggerCallback, this);

  cmdloop_timer_    = nh_.createTimer(ros::Duration(0.01), &attitudeCtrl::cmdloopCallback, this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &attitudeCtrl::statusloopCallback, this);  // Define timer for constant loop rate

  angularVelPub_    = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_  = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_   = nh_.advertise<nav_msgs::Path>("attitude_controller/path", 10);
  
  // referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);

  // 发布 Mavros 消息
  systemstatusPub_  = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1); // mavlink 状态
  arming_client_    = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_  = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_     = nh_.advertiseService("land", &attitudeCtrl::landCallback, this);

  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);

  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);

  nh_private_.param<double>("Kw_xy", kw_xy_, 1.0);
  nh_private_.param<double>("Kw_z", kw_z_, 1.0);

  nh_private_.param<double>("Kw_xy_fb", kw_xy_ff_, 0.5);
  nh_private_.param<double>("Kw_z_fb", kw_z_ff_, 0.5);

  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);

  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

  D_ << dx_, dy_, dz_;

  tau << tau_x, tau_y, tau_z;

  targetYawdot_ = 0;
  targetYaw_ = 0;
}
attitudeCtrl::~attitudeCtrl() {
  // Destructor
}


// 目标角速度、速度、加速度
void attitudeCtrl::targetCallback(const geometry_msgs::TwistStamped &msg) {
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if (reference_request_dt_ > 0)
	targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
	targetAcc_ = Eigen::Vector3d::Zero();
}
// 目标位置、速度、加速度
// void attitudeCtrl::flattargetCallback(const quadrotor_msgs::FlatTarget &msg) {
  
//   node_state = MISSION_EXECUTION;

//   reference_request_last_ = reference_request_now_;

//   targetPos_prev_ = targetPos_;
//   targetVel_prev_ = targetVel_;

//   reference_request_now_ = ros::Time::now();
//   reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

//   targetPos_ = toEigen(msg.position);
//   targetVel_ = toEigen(msg.velocity);

//   targetYaw_ = msg.yaw;
//   targetYawdot_ = msg.yaw_dot;
//   // targetYaw_ = 0;
//   // targetYawdot_ = 0;
//   if (msg.type_mask == 1) {
// 	targetAcc_ = toEigen(msg.acceleration);
// 	targetJerk_ = toEigen(msg.jerk);
// 	targetSnap_ = Eigen::Vector3d::Zero();

//   } else if (msg.type_mask == 2) {
// 	targetAcc_ = Eigen::Vector3d::Zero();
// 	targetJerk_ = Eigen::Vector3d::Zero();
// 	targetSnap_ = Eigen::Vector3d::Zero();

//   } else if (msg.type_mask == 4) {
// 	targetAcc_ = Eigen::Vector3d::Zero();
// 	targetJerk_ = Eigen::Vector3d::Zero();
// 	targetSnap_ = Eigen::Vector3d::Zero();

//   } else {
// 	targetAcc_ = toEigen(msg.acceleration);
// 	targetJerk_ = toEigen(msg.jerk);
// 	targetSnap_ = toEigen(msg.snap);
//   }
// }

// 无人机当前的姿态（四元数）
void attitudeCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
	received_home_pose = true;
	home_pose_ = msg.pose;
	ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position);
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
}
// 无人机当前的线速度和角速度
void attitudeCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}

bool attitudeCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  node_state = LANDING;
}
//接受无人机控制指令
void attitudeCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
	case WAITING_FOR_HOME_POSE:
	  waitForPredicate(&received_home_pose, "Waiting for home pose...");
	  ROS_INFO("Got pose! Drone Ready to be armed.");
	  node_state = TAKING_OFF;
	  break;

  case TAKING_OFF:
  {
    geometry_msgs::PoseStamped takingoff_msg;
    takingoff_msg.header.stamp = ros::Time::now();
    takingoff_msg.pose.position.x = initTargetPos_x_;
    takingoff_msg.pose.position.y = initTargetPos_y_;
    takingoff_msg.pose.position.z = initTargetPos_z_;
    takingoff_msg.pose.orientation.w = mavAtt_(0);
    takingoff_msg.pose.orientation.x = mavAtt_(1);
    takingoff_msg.pose.orientation.y = mavAtt_(2);
    takingoff_msg.pose.orientation.z = mavAtt_(3);
    target_pose_pub_.publish(takingoff_msg);
    ros::spinOnce();
    break;
  }

	case MISSION_EXECUTION:
	  if (!feedthrough_enable_) 
      computeBodyRateCmd(cmdBodyRate_, targetPos_, targetVel_, targetAcc_);
	  pubReferencePose(targetPos_, q_des);
	  pubRateCommands(cmdBodyRate_);
	  appendPoseHistory();
	  pubPoseHistory();
	  break;

	case LANDING: {
    if (autoland())
      node_state = LANDED;
    ros::spinOnce();
    break;
	}
	case LANDED:
	  ROS_INFO("Landed. Please set to position control and disarm.");
	  cmdloop_timer_.stop();
	  break;
  }
}

bool attitudeCtrl::autoland()
{
  geometry_msgs::PoseStamped landingmsg;
  if (mavPos_(2) <= 0.3)
  {
    if (current_state_.mode != "AUTO.LAND")
    {
      offb_set_mode_.request.custom_mode = "AUTO.LAND";
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
      {
        ROS_INFO("AUTO.LAN enabled");
        return true;
      }
    }
  }
  else
  {
    landingmsg.header.stamp = ros::Time::now();
    landingmsg.pose.position.z = 0.15;
    landingmsg.pose.position.x = mavPos_(0);
    landingmsg.pose.position.y = mavPos_(1);
    landingmsg.pose.orientation.w = mavAtt_(0);
    landingmsg.pose.orientation.x = mavAtt_(1);
    landingmsg.pose.orientation.y = mavAtt_(2);
    landingmsg.pose.orientation.z = mavAtt_(3);
    target_pose_pub_.publish(landingmsg);
  }
  return false;
}


//接受无人机当前的状态
void attitudeCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }
// 无人机解锁及切换模式
void attitudeCtrl::statusloopCallback(const ros::TimerEvent &event) {
  if (sim_enable_) {
	// Enable OFFBoard mode and arm automatically
	// This is only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(2.0)))
    {
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    }
    else
    {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(2.0)))
      {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void attitudeCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void attitudeCtrl::pubRateCommands(const Eigen::Vector4d &cmd) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128;  // Ignore orientation messages
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

void attitudeCtrl::pubPoseHistory() {
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;

  posehistoryPub_.publish(msg);
}

void attitudeCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void attitudeCtrl::appendPoseHistory() {
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_) {
	posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped attitudeCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
																  Eigen::Vector4d &orientation) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

void attitudeCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_pos,
									   const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc) {

  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d body_rotmat;
  Eigen::Vector3d zb, zb_des, yb_des, xb_des;
	// Eigen::Vector4d ratecmd;	

  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
	mavYaw_ = getVelocityYaw(mavVel_);
  }

  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;

  // feedforward term for trajectory error
  Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  

  if (a_fb.norm() > max_fb_acc_)
	  a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large


  const Eigen::Vector3d a_des = a_fb + a_ref - g_;

  /* ***************** 计算机体zb  ***************** */
  body_rotmat = quat2RotMatrix(mavAtt_);
  zb = body_rotmat.col(2);

  /* ***************** 计算推力  ***************** */
  bodyrate_cmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * a_des.dot(zb) + norm_thrust_offset_));  // Calculate thrust

  //  std::cout << "a_fb.norm(): " << a_fb.norm() << std::endl;
  // std::cout << "bodyrate_cmd(3): " << bodyrate_cmd(3) << std::endl;

  /* ***************** 计算角速度前馈控制量 ***************** */
  Eigen::Vector3d z_ref, y_ref, x_ref, proj_x_ref ,a_ref_g, zw;
  zw << 0, 0, 1;
  proj_x_ref << std::cos(targetYaw_), std::sin(targetYaw_), 0.0;
  a_ref_g = a_ref - g_;
  z_ref = (a_ref_g) / (a_ref_g).norm();
  y_ref = z_ref.cross(proj_x_ref) / (z_ref.cross(proj_x_ref)).norm();
  x_ref = y_ref.cross(z_ref) / (y_ref.cross(z_ref)).norm();

  Eigen::Vector3d w_ref;
  w_ref(0) = - y_ref.dot(targetJerk_)/a_ref_g.norm();
  w_ref(1) =   x_ref.dot(targetJerk_)/a_ref_g.norm();
  w_ref(2) = targetYawdot_*(zw.dot(z_ref));
  // w_ref(2) = 0;

  /* ***************** 计算期望的俯仰和横滚角速率 ***************** */
  zb_des = a_des / a_des.norm();

  // std::cout << "zb_des.norm(): " << zb_des.norm() << std::endl;
  // std::cout << "zb.norm(): " << zb.norm() << std::endl;
  double alpha = std::acos(zb.dot(zb_des));
  // std::cout << "alpha: " << alpha << std::endl;
  if(alpha==0){

  }
  Eigen::Vector3d n, n_b;
  n = zb.cross(zb_des) / (zb.cross(zb_des)).norm();
  n_b = quaternion_rotate_vector(quaternion_inv(mavAtt_),n);

  n_b = n_b * std::sin(alpha/2.0);

  Eigen::Vector4d qe_xy;
  qe_xy << std::cos(alpha/2.0),n_b(0),n_b(1),n_b(2);
  // std::cout << "qe_xy: " << qe_xy(0) << " " << qe_xy(1) << " " << qe_xy(2) << " " << qe_xy(3) << " " << std::endl;
  if(qe_xy(0) >= 0){
    bodyrate_cmd(0) = 2*kw_xy_*qe_xy(1) - kw_xy_ff_*w_ref(0);
    bodyrate_cmd(1) = 2*kw_xy_*qe_xy(2) - kw_xy_ff_*w_ref(1);
    // std::cout << "\033[31m qe_xy(0) >= 0: " << w_ref(0) << " " << w_ref(1) << " " << w_ref(2) << " \033[0m" << std::endl;
  }else{
    bodyrate_cmd(0) = -2*kw_xy_*qe_xy(1) - kw_xy_ff_*w_ref(0);
    bodyrate_cmd(1) = -2*kw_xy_*qe_xy(2) - kw_xy_ff_*w_ref(1);
    // std::cout << "\033[32m qe_xy(0) < 0: " << w_ref(0) << " " << w_ref(1) << " " << w_ref(2) << " \033[0m" << std::endl;
  }
  
  /* ***************** 计算期望的偏航角速率 ***************** */
  Eigen::Vector3d xc, yc;

  // xc << std::cos(targetYaw_), std::sin(targetYaw_), 0;
  yc << -std::sin(targetYaw_), std::cos(targetYaw_), 0;

  Eigen::Vector3d xb_temp = yc.cross(zb_des);
  if(xb_temp.sum()==0){
    bodyrate_cmd(2) = 0;
    std::cout << "xb_temp.sum() = 0" << std::endl;
    return;
  }
  xb_des = xb_temp / xb_temp.norm();

  // If the desired body z axis has a negative z component
  // if(){

  // }
  
  yb_des = zb_des.cross(xb_des) / (zb_des.cross(xb_des)).norm();

  Eigen::Matrix3d rotmat_des;
  rotmat_des << xb_des(0), yb_des(0), zb_des(0), 
                xb_des(1), yb_des(1), zb_des(1), 
                xb_des(2), yb_des(2), zb_des(2);

  q_des = rot2Quaternion(rotmat_des);

  Eigen::Vector4d q_temp = quatMultiplication(mavAtt_,qe_xy);

  q_temp = quaternion_inv(q_temp);

  Eigen::Vector4d qe_z = quatMultiplication(q_temp,q_des);
  // std::cout << "qe_z: " << qe_z(0) << " " << qe_z(1) << " " << qe_z(2) << " " << qe_z(3) << " " << std::endl;
  if(qe_z(0) >= 0){
    bodyrate_cmd(2) = 2*kw_z_*qe_z(3) + kw_z_ff_*w_ref(2);
  }else{
    bodyrate_cmd(2) = -2*kw_z_*qe_z(3) + kw_z_ff_*w_ref(2);
  }

  // std::cout << "qe_xy: "<< bodyrate_cmd(0) << " "  << bodyrate_cmd(1) << " " << bodyrate_cmd(2) << " " << std::endl;
  
}

Eigen::Vector4d attitudeCtrl::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), 
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
	        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), 
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

// 四元数->旋转矩阵
Eigen::Matrix3d attitudeCtrl::quat2RotMatrix(const Eigen::Vector4d &q) {
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
	  2 * q(0) * q(2) + 2 * q(1) * q(3),

	  2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
	  2 * q(2) * q(3) - 2 * q(0) * q(1),

	  2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
	  q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

// 旋转矩阵->四元数
Eigen::Vector4d attitudeCtrl::rot2Quaternion(const Eigen::Matrix3d &R) {
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
	double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
	quat(0) = 0.25 * S;
	quat(1) = (R(2, 1) - R(1, 2)) / S;
	quat(2) = (R(0, 2) - R(2, 0)) / S;
	quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
	double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
	quat(0) = (R(2, 1) - R(1, 2)) / S;
	quat(1) = 0.25 * S;
	quat(2) = (R(0, 1) + R(1, 0)) / S;
	quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
	double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
	quat(0) = (R(0, 2) - R(2, 0)) / S;
	quat(1) = (R(0, 1) + R(1, 0)) / S;
	quat(2) = 0.25 * S;
	quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
	double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
	quat(0) = (R(1, 0) - R(0, 1)) / S;
	quat(1) = (R(0, 2) + R(2, 0)) / S;
	quat(2) = (R(1, 2) + R(2, 1)) / S;
	quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector4d attitudeCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

double attitudeCtrl::getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); }


// 四元数旋转向量
Eigen::Vector3d attitudeCtrl::quaternion_rotate_vector(const Eigen::Vector4d &q,const Eigen::Vector3d &v) {
  Eigen::Vector4d q_conjugate, p, q_temp;
  q_conjugate << q(0),-q(1),-q(2),-q(3);
  p << 0, v(0),v(1),v(2);
  q_temp = quatMultiplication(q,p);
  q_temp = quatMultiplication(q_temp,q_conjugate);

  Eigen::Vector3d v_r;
  v_r << q_temp(1),q_temp(2),q_temp(3);
  return v_r;
}

// 四元数的逆
Eigen::Vector4d attitudeCtrl::quaternion_inv(const Eigen::Vector4d &q) {
  Eigen::Vector4d q_conjugate, q_inv;
  q_conjugate << q(0),-q(1),-q(2),-q(3);
  q_inv = q_conjugate/q.norm();
  return q_inv;
}

// 向量 -> 斜对称矩阵
Eigen::Matrix3d attitudeCtrl::matrix_hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  // Sanity checks on M
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}

// 斜对称矩阵 -> 向量
Eigen::Vector3d attitudeCtrl::matrix_hat_inv(const Eigen::Matrix3d &m) {
  Eigen::Vector3d v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

void attitudeCtrl::getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel,
							  Eigen::Vector3d &angvel) {
  pos = mavPos_;
  att = mavAtt_;
  vel = mavVel_;
  angvel = mavRate_;
}

void attitudeCtrl::getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
  pos = mavPos_ - targetPos_;
  vel = mavVel_ - targetVel_;
}

bool attitudeCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
}

void attitudeCtrl::setBodyRateCommand(Eigen::Vector4d bodyrate_command) { cmdBodyRate_ = bodyrate_command; }

void attitudeCtrl::setFeedthrough(bool feed_through) { feedthrough_enable_ = feed_through; }

void attitudeCtrl::dynamicReconfigureCallback(attitude_controller::AttitudeControllerConfig &config,
											   uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
	max_fb_acc_ = config.max_acc;
	ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  } else if (Kpos_x_ != config.Kp_x) {
	Kpos_x_ = config.Kp_x;
	ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
	Kpos_y_ = config.Kp_y;
	ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
	Kpos_z_ = config.Kp_z;
	ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
	Kvel_x_ = config.Kv_x;
	ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
	Kvel_y_ = config.Kv_y;
	ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
	Kvel_z_ = config.Kv_z;
	ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  } else if (kw_xy_ != config.Kw_xy) {
	kw_xy_ = config.Kw_xy;
	ROS_INFO("Reconfigure request : Kw_xy =%.2f  ", config.Kw_xy);
  } else if (kw_z_ != config.Kw_z) {
	kw_z_ = config.Kw_z;
	ROS_INFO("Reconfigure request : Kw_z  = %.2f  ", config.Kw_z);
  } else if (kw_xy_ff_ != config.Kw_xy_ff) {
	kw_xy_ff_ = config.Kw_xy_ff;
	ROS_INFO("Reconfigure request : Kw_xy_ff  = %.2f  ", config.Kw_xy_ff);
  } else if (kw_z_ff_ != config.Kw_z_ff) {
	kw_z_ff_ = config.Kw_z_ff;
	ROS_INFO("Reconfigure request : Kw_z_ff  = %.2f  ", config.Kw_z_ff);
  } else if (norm_thrust_const_ != config.thrust_constant) {
	norm_thrust_const_ = config.thrust_constant;
	ROS_INFO("Reconfigure request : thrust_constant =%.2f  ", config.thrust_constant);
  } else if (norm_thrust_offset_ != config.thrust_offset) {
	norm_thrust_offset_ = config.thrust_offset;
	ROS_INFO("Reconfigure request : thrust_offset  = %.2f  ", config.thrust_offset);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_z_, -Kvel_z_;
}
