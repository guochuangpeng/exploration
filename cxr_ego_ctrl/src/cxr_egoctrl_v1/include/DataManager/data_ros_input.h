#ifndef __DATA_ROS_INPUT_H__
#define __DATA_ROS_INPUT_H__

#include <DataManager/common.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>

#include "quadrotor_msgs/PositionCommand.h"
class DataRosInput {
 public:
  DataRosInput(ros::NodeHandle &nh);
  ~DataRosInput() = default;

  void init();

  bool getUavState(mavros_msgs::State &uav_state) {
    bool ret = false;
    uav_state_lock_.lock();
    if (is_uav_state_info_updated_) {
      uav_state = uav_state_;
      is_uav_state_info_updated_ = false;
      ret = true;
    }
    uav_state_lock_.unlock();
    return ret;
  };
  bool getOdomInfo(UavState &odom) {
    bool ret = false;
    odom_lock_.lock();
    if (is_odom_info_updated_) {
      odom = odom_;
      is_odom_info_updated_ = false;
      ret = true;
    }
    odom_lock_.unlock();
    return ret;
  };

  bool getPlanState(UavState &desire_state) {
    bool ret = false;
    planner_lock_.lock();
    if (is_planner_info_updated_) {
      desire_state = desire_state_;
      is_planner_info_updated_ = false;
      ret = true;
    }
    planner_lock_.unlock();
    return ret;
  }
  bool getPlanStateQueue(std::vector<UavState> &desire_states) {
    bool ret = false;
    planner_lock_.lock();
    if (is_planner_info_updated_) {
      desire_states.push_back(desire_state_);
      is_planner_info_updated_ = false;
      ret = true;
    }
    planner_lock_.unlock();
    return ret;
  }
  bool getPlannerState(std::string &planner_state) {
    bool ret = false;
    planner_state_lock.lock();
    if (is_planner_state_info_updated_) {
      planner_state=planner_state_;
      is_planner_state_info_updated_ = false;
      ret = true;
    }
    planner_state_lock.unlock();
    return ret;
  }

 private:
  /******回调函数*************/
  void plannerStateCallback(
      const std_msgs::String::ConstPtr &planner_state_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void uavStateCallback(const mavros_msgs::State::ConstPtr &uav_state_msg);
  void plannerCallback(
      const quadrotor_msgs::PositionCommand::ConstPtr &planner_msg);
  // ros 订阅器
 private:
  ros::NodeHandle nh_;
  ros::Subscriber uav_state_sub_;
  ros::Subscriber rc_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber planner_sub_;
  ros::Subscriber planner_state_sub_;

  // 无人机信息
 private:
  UavState odom_;
  mavros_msgs::State uav_state_;
  UavState desire_state_;
  std::string planner_state_;

  // 锁
 private:
  std::mutex odom_lock_;
  std::mutex uav_state_lock_;
  std::mutex planner_lock_;
  std::mutex planner_state_lock;
  // 更新判断
 private:
  bool is_odom_info_updated_{false};
  bool is_uav_state_info_updated_{false};
  bool is_planner_info_updated_{false};
  bool is_planner_state_info_updated_{false};
};
#endif