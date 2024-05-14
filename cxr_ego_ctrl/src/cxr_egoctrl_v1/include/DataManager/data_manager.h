#ifndef __DATA_MANAGER_H__
#define __DATA_MANAGER_H__
#include <DataManager/common.h>
#include <DataManager/data_ros_input.h>
#include <DataManager/data_ros_output.h>
#include <ros/ros.h>

#include <memory>
#include <queue>
enum class FlightState { TAKEOFF, SPIN, FOLLOW_TRAJECTORY,Landing,FINISH };
class DataManager {
 public:
  DataManager(ros::NodeHandle &nh);
  ~DataManager() = default;
  void Execute();
  void update();
  void takeoff();
  void spin();
  void landing();

 private:
  ros::NodeHandle nh_;
  int SIM_OR_REAL;
  std::shared_ptr<DataRosInput> data_input_;
  std::shared_ptr<DataRosOutput> data_output_;
  FlightState current_fsm_;
  ros::Time start_time_;
  mavros_msgs::State cur_state_;
  UavState last_desire_uav_state_;
  UavState desire_uav_state_;
  UavState odom_;
  std::queue<UavState> plan_queue_;
  std::string planner_state_{"INIT"};

  bool is_spin_success_{false};
};
#endif