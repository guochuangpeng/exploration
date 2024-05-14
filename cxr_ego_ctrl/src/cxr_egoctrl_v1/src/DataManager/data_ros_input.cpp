#include <DataManager/data_ros_input.h>
DataRosInput::DataRosInput(ros::NodeHandle &nh) : nh_(nh) { init(); };
void DataRosInput::init() {
  uav_state_sub_ = nh_.subscribe<mavros_msgs::State>(
      "/iris_0/mavros/state", 10, &DataRosInput::uavStateCallback, this);
  odom_sub_ =
      nh_.subscribe<nav_msgs::Odometry>("/iris_0/mavros/local_position/odom",
                                        10, &DataRosInput::odomCallback, this);
  planner_sub_ = nh_.subscribe<quadrotor_msgs::PositionCommand>(
      "/planning/pos_cmd", 10, &DataRosInput::plannerCallback, this);
  planner_state_sub_ = nh_.subscribe<std_msgs::String>(
      "/planner_state", 10, &DataRosInput::plannerStateCallback, this);
}
void DataRosInput::plannerStateCallback(
    const std_msgs::String::ConstPtr &planner_state_msg) {
  planner_state_lock.lock();
  planner_state_ = planner_state_msg->data;
  is_planner_state_info_updated_=true;
  planner_state_lock.unlock();
}
void DataRosInput::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  odom_lock_.lock();
  odom_.pos = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                              odom_msg->pose.pose.position.y,
                              odom_msg->pose.pose.position.z);
  odom_.vel = Eigen::Vector3d(odom_msg->twist.twist.linear.x,
                              odom_msg->twist.twist.linear.y,
                              odom_msg->twist.twist.linear.z);
  odom_.acc = Eigen::Vector3d(0.0, 0.0, 0.0);
  odom_.q = Eigen::Quaterniond(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  odom_.yaw = quaternion_to_ypr(Eigen::Quaternion<double>(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z))(0);

  is_odom_info_updated_ = true;
  odom_lock_.unlock();
}
void DataRosInput::uavStateCallback(
    const mavros_msgs::State::ConstPtr &uav_state_msg) {
  uav_state_lock_.lock();
  uav_state_ = *uav_state_msg;
  is_uav_state_info_updated_ = true;
  uav_state_lock_.unlock();
}
void DataRosInput::plannerCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr &planner_msg) {
  planner_lock_.lock();
  desire_state_.pos =
      Eigen::Vector3d(planner_msg->position.x, planner_msg->position.y,
                      planner_msg->position.z);
  desire_state_.vel =
      Eigen::Vector3d(planner_msg->velocity.x, planner_msg->velocity.y,
                      planner_msg->velocity.z);
  desire_state_.acc =
      Eigen::Vector3d(planner_msg->acceleration.x, planner_msg->acceleration.y,
                      planner_msg->acceleration.z);
  desire_state_.yaw = planner_msg->yaw;
  double yaw = planner_msg->yaw;
  desire_state_.q = yaw_to_quaternion(yaw);
  is_planner_info_updated_ = true;
  planner_lock_.unlock();
}