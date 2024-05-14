#include <DataManager/data_ros_output.h>
DataRosOutput::DataRosOutput(ros::NodeHandle &nh) : nh_(nh) { init(); };
void DataRosOutput::init() {
  position_yaw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/iris_0/mavros/setpoint_position/local", 1);

  arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");

  ros::ServiceClient set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
  trigger_pub_ =
      nh_.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);
  landing_client_ =
      nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
}
void DataRosOutput::sendDesireState(const UavState &desire_state) {
  geometry_msgs::PoseStamped pos_pub;
  pos_pub.pose.position.x = desire_state.pos(0);
  pos_pub.pose.position.y = desire_state.pos(1);
  pos_pub.pose.position.z = desire_state.pos(2);
  pos_pub.pose.orientation.x = desire_state.q.x();
  pos_pub.pose.orientation.y = desire_state.q.y();
  pos_pub.pose.orientation.z = desire_state.q.z();
  pos_pub.pose.orientation.w = desire_state.q.w();
  publishPositionYaw(pos_pub);
}