#ifndef __DATA_ROS_OUTPUT_H__
#define __DATA_ROS_OUTPUT_H__
#include <DataManager/common.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandBoolRequest.h>
#include <mavros_msgs/CommandBoolResponse.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetModeRequest.h>
#include <mavros_msgs/SetModeResponse.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
class DataRosOutput {
 public:
  DataRosOutput(ros::NodeHandle &nh);
  ~DataRosOutput() = default;
  void init();
  void sendDesireState(const UavState &desire_state);
  void sendArmingCommand() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("Vehicle armed");
    } else {
      ROS_ERROR("Failed to arm vehicle");
    }
  }
  void sendOffboradCommand() {
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client_.call(offboard_set_mode)) {
      ROS_INFO("Offboard enabled");
    } else {
      ROS_ERROR("Failed to set offboard mode");
    }
  }
  void sendLandingCommand(){
     mavros_msgs::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 0;
    landing_cmd.request.yaw = 0;
    landing_cmd.request.latitude = 0;
    landing_cmd.request.longitude = 0;
    landing_cmd.request.altitude = 0;

    if (landing_client_.call(landing_cmd))
    {
        if (landing_cmd.response.success)
        {
            ROS_INFO("Landing command sent successfully");
        }
        else
        {
            ROS_ERROR("Failed to send landing command");
        }
    }
    else
    {
        ROS_ERROR("Failed to call landing service");
    }
  }
  void sendTriggerSignal() {
    nav_msgs::Path trigger_signal;
    trigger_signal.header.seq=000;
    geometry_msgs::PoseStamped trigger_pose;
    trigger_pose.pose.position.x=0;
    trigger_pose.pose.position.y=0;
    trigger_pose.pose.position.z=0;

    std::vector<geometry_msgs::PoseStamped> posehistory_vector;
    posehistory_vector.push_back(trigger_pose);
    trigger_signal.poses=posehistory_vector;
 
    trigger_pub_.publish(trigger_signal);
  }
  void publishPositionYaw(const geometry_msgs::PoseStamped &data) {
    position_yaw_pub_.publish(data);
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher position_yaw_pub_;
  ros::Publisher trigger_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient landing_client_;

};
#endif