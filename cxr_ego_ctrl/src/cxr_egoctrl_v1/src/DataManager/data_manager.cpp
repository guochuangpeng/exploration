#include <DataManager/data_manager.h>

DataManager::DataManager(ros::NodeHandle &nh) : nh_(nh) {
  data_input_ = std::make_shared<DataRosInput>(nh_);
  data_output_ = std::make_shared<DataRosOutput>(nh_);
  ros::param::get("sim_or_real", SIM_OR_REAL);
}

void DataManager::update() {
  data_input_->getUavState(cur_state_);
  if (cur_state_.connected == false) {
    ROS_ERROR("未更新无人机状态，可能存在断连");
  } else {
    if (cur_state_.mode != "OFFBOARD" && current_fsm_ != FlightState::FINISH) {
      current_fsm_ = FlightState::TAKEOFF;
    }
  }

  if (data_input_->getPlannerState(planner_state_)) {
    ROS_INFO("规划器状态：%s", planner_state_.c_str());
    if (planner_state_.compare("FINISH_PLANNER") == 0) {
      current_fsm_ = FlightState::Landing;
    }
  }

  if (data_input_->getPlanState(desire_uav_state_) &&
      (last_desire_uav_state_.pos(0) != desire_uav_state_.pos(0) ||
       last_desire_uav_state_.pos(1) != desire_uav_state_.pos(1) ||
       last_desire_uav_state_.pos(2) != desire_uav_state_.pos(2))) {
    plan_queue_.push(desire_uav_state_);
    last_desire_uav_state_ = desire_uav_state_;
  }
  ROS_WARN("队列大小：%d", plan_queue_.size());

  switch (current_fsm_) {
    case FlightState::TAKEOFF:
      takeoff();
      ROS_WARN("current_fsm_：：TAKEOFF");
      break;
    case FlightState::SPIN:
      spin();
      ROS_WARN("current_fsm_：：SPIN");
      break;
    case FlightState::FOLLOW_TRAJECTORY:
      Execute();
      ROS_WARN("current_fsm_：：FOLLOW_TRAJECTORY");
      break;
    case FlightState::Landing:
      landing();
      ROS_WARN("current_fsm_：：Landing");
      break;
    default:
      break;
  }
}
void DataManager::landing() {
  data_output_->sendLandingCommand();
  current_fsm_ = FlightState::FINISH;
}
void DataManager::spin() {
  if (!plan_queue_.empty() && is_spin_success_) {
    current_fsm_ = FlightState::FOLLOW_TRAJECTORY;
  }
  double T = 30;  // 自旋速度
  //   mavros_msgs::State cur_state;
  //   data_input_->getUavState(cur_state);
  data_input_->getOdomInfo(odom_);
  UavState takeoff_state;
  double yaw = (ros::Time::now() - start_time_).toSec() * T * M_PI / 180.0;
  takeoff_state.q.w() = cos(yaw / 2.0);
  takeoff_state.q.x() = 0.0;
  takeoff_state.q.y() = 0.0;
  takeoff_state.q.z() = sin(yaw / 2.0);
  takeoff_state.pos = odom_.pos;
  if ((ros::Time::now() - start_time_).toSec() * T >= 360) {
    is_spin_success_ = true;
    data_output_->sendTriggerSignal();
  }

  data_output_->sendDesireState(takeoff_state);
}
void DataManager::takeoff() {
  data_input_->getOdomInfo(odom_);
  mavros_msgs::State cur_state;
  data_input_->getUavState(cur_state);
  UavState takeoff_state;
  takeoff_state.pos = Eigen::Vector3d(odom_.pos(0), odom_.pos(1), 2);
  data_output_->sendDesireState(takeoff_state);
  if (SIM_OR_REAL == 0) {
    if (cur_state_.armed != true) {
      data_output_->sendArmingCommand();
    }
    if (cur_state_.mode != "OFFBOARD") {
      data_output_->sendOffboradCommand();
    }

  } else {
    ROS_WARN("实际飞行，需要手动解锁并切换OFFBOARD模式");
  }

  if ((odom_.pos - takeoff_state.pos).norm() < 0.5) {
    ROS_INFO("起飞成功");
    current_fsm_ = FlightState::SPIN;
    start_time_ = ros::Time::now();
  }
}

void DataManager::Execute() {
  mavros_msgs::State cur_state;
  data_input_->getUavState(cur_state);
  data_input_->getOdomInfo(odom_);

  if (plan_queue_.empty()) {
    ROS_INFO("规划结束或没有数据");
    // current_fsm_ = FlightState::SPIN;
    data_output_->sendDesireState(last_desire_uav_state_);
    start_time_ = ros::Time::now();
    return;
  }
  UavState cur_uav_state = plan_queue_.front();
  if ((cur_uav_state.pos - odom_.pos).norm() < 0.3) {
    plan_queue_.pop();
  }

  ROS_INFO("cur_uav_state: x = %.2f m, y = %.2f m, z = %.2f m",
           cur_uav_state.pos(0), cur_uav_state.pos(1), cur_uav_state.pos(2));
  data_output_->sendDesireState(cur_uav_state);

  //   } else {
  //     ROS_INFO("没有接收到轨迹");
  //     // current_fsm_ = FlightState::SPIN;
  //   }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "autoPlanner");
  setlocale(LC_ALL, "");
  ros::NodeHandle nh;
  std::shared_ptr<DataManager> data_manager = std::make_shared<DataManager>(nh);
  ros::Rate rate(150.0);
  ROS_INFO("启动");
  while (ros::ok()) {
    data_manager->update();
    ros::spinOnce();
    rate.sleep();
  }
}
