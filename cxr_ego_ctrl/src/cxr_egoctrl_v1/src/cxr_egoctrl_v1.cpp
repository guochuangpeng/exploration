/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>
#include "Position_Controller/pos_controller_PID.h"

#define VELOCITY2D_CONTROL 0b101111000111 // 设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
// 设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
visualization_msgs::Marker trackpoint;
ros::Publisher *pubMarkerPointer;
unsigned short velocity_mask = VELOCITY2D_CONTROL;
mavros_msgs::PositionTarget current_goal;
mavros_msgs::RCIn rc;
int rc_value, flag = 0, flag1 = 0;
nav_msgs::Odometry position_msg;
geometry_msgs::PoseStamped target_pos;
mavros_msgs::State current_state;
float position_x, position_y, position_z, current_yaw, targetpos_x, targetpos_y, vel_x, vel_y, vel_z;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; // EGO planner information has position velocity acceleration yaw yaw_dot
bool receive = false;																									  // 触发轨迹的条件判断
float pi = 3.14159265;
bool isOffboard = false;
std::queue<quadrotor_msgs::PositionCommand> positionCommandQueue;
pos_controller_PID posPID;
nav_msgs::Odometry odom;
mavros_msgs::State state;
// read RC 5 channel pwm,estimate auto plan in certain range
void rc_cb(const mavros_msgs::RCIn::ConstPtr &msg)
{
	rc = *msg;
	rc_value = rc.channels[4];
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
	if (current_state.mode == "OFFBOARD")
	{
		isOffboard = true;
	}

	state = *msg;
}

// read vehicle odometry
void position_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	position_msg = *msg;
	position_x = position_msg.pose.pose.position.x;
	position_y = position_msg.pose.pose.position.y;
	position_z = position_msg.pose.pose.position.z;
	vel_x = position_msg.twist.twist.linear.x;
	vel_y = position_msg.twist.twist.linear.y;
	vel_z = position_msg.twist.twist.linear.z;

	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); // 把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	current_yaw = yaw;

	odom = *msg;
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) // 读取rviz的航点
{
	target_pos = *msg;
	targetpos_x = target_pos.pose.position.x;
	targetpos_y = target_pos.pose.position.y;
}

// 读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
quadrotor_msgs::PositionCommand ego;
void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg) // ego的回调函数
{

	ego = *msg;
	// positionCommandQueue.push(cur_ego);
	ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;

	ego_vel_x = ego.velocity.x;
	ego_vel_y = ego.velocity.y;
	ego_vel_z = ego.velocity.z;
	ego_a_x = ego.acceleration.x;
	ego_a_y = ego.acceleration.y;
	ego_a_y = ego.acceleration.y;
	ego_yaw = ego.yaw;
	ego_yaw_rate = ego.yaw_dot;
	receive = true;
}
ros::Publisher px4_setpoint_raw_attitude_pub;
void send_attitude_setpoint(Eigen::Vector4d &u_att)
{
	mavros_msgs::AttitudeTarget att_setpoint;
	// Mappings: If any of these bits are set, the corresponding input should be ignored:
	// bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
	//  0b00000111;
	att_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
							 mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
							 mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	Eigen::Vector3d att_des;
	att_des << u_att(0), u_att(1), u_att(2);
	Eigen::Quaterniond q_des = quaternion_from_rpy(att_des);
	att_setpoint.orientation.x = q_des.x();
	att_setpoint.orientation.y = q_des.y();
	att_setpoint.orientation.z = q_des.z();
	att_setpoint.orientation.w = q_des.w();
	att_setpoint.thrust = u_att(3);
	px4_setpoint_raw_attitude_pub.publish(att_setpoint);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cxr_egoctrl_v1");
	setlocale(LC_ALL, "");
	ros::NodeHandle nh;
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 10, state_cb);				  // 读取飞控状态的话题
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 1); // 这个话题很重要，可以控制无人机的位置速度加速度和yaw以及yaw-rate，里面有个掩码选择，需要注意

	ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
	pubMarkerPointer = &pubMarker;
	px4_setpoint_raw_attitude_pub =
		nh.advertise<mavros_msgs::AttitudeTarget>("/iris_0/mavros/setpoint_raw/attitude", 1);
	ros::Publisher px4_setpoint_position_pub =
		nh.advertise<geometry_msgs::PoseStamped>("/iris_0/mavros/setpoint_position/local", 1);

	ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/iris_0/mavros/rc/in", 10, rc_cb);				// 读取遥控器通道的话题，目前不需要
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming"); // 控制无人机解锁的服务端，不需要用
	ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>("/iris_0/mavros/cmd/command");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");		  // 设置飞机飞行模式的服务端，也不需要用
	ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, twist_cb); // 订阅egoplanner的规划指令话题的
	ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 10, target_cb);
	ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/iris_0/mavros/local_position/odom", 10, position_cb);
	ros::Rate rate(150.0); // 控制频率尽可能高点，大于30hz

	posPID.init(nh);

	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;				// 用不上
	offb_set_mode.request.custom_mode = "OFFBOARD"; // 用不上
	mavros_msgs::CommandBool arm_cmd;				// 用不上
	arm_cmd.request.value = true;					// 用不上
	// send a few setpoints before starting
	// for (int i = 100; ros::ok() && i > 0; --i)
	// {
	// 	current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	// 	local_pos_pub.publish(current_goal);
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }
	while (ros::ok() /*&&rc_value>900&&rc_value<1150*/)
	{
		// if( current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0) && flag1 == 0){
		// 			if( set_mode_client.call(offb_set_mode) &&	offb_set_mode.response.mode_sent){
		// 				ROS_INFO("已进入Offboard模式，等待两秒解锁");
		// 				flag1++;
		// 			}
		// 			last_request = ros::Time::now();
		// 		}
		// 	else {
		// 			if( !current_state.armed &&ros::Time::now() - last_request > ros::Duration(2.0) && flag1 == 1)
		// 			{
		// 				if( arming_client.call(arm_cmd) &&arm_cmd.response.success)
		// 				{
		// 					ROS_INFO("已解锁，起飞");
		// 					flag1++;
		// 				}
		// 				last_request = ros::Time::now();
		// 			}
		// 		}

		// take off 1m
		if (!receive && isOffboard) // 如果没有在rviz上打点，则offboard模式下会保持在0，0，1的高度
		{
			ROS_INFO("OFFboard模式");
			// 设置目标位置为起飞高度（例如：3米）
			double takeoff_altitude = 3.0;
			// current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			current_goal.header.frame_id = "world";
			current_goal.header.stamp = ros::Time::now();
			current_goal.type_mask = 0b10111111000; // 设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
			// 设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
			// current_goal.type_mask = 1479;
			// 设置起飞速度（可以根据需要调整）
			double takeoff_velocity_z = 1.0;
			current_goal.position.x = 0;
			current_goal.position.y = 0;
			current_goal.position.z = 2;
			// 垂直上升速度
			// 设置自旋的偏航速度（可以根据需要调整）
			double yaw_rate = 1.5; // 弧度/秒
			// current_goal.yaw = 3.14 / 3;
			current_goal.yaw_rate = yaw_rate;
			// 让无人机起飞并自旋一段时间（例如：5秒）
			// ros::Duration(5.0).sleep();
			// 停止自旋和上升
			// current_goal.velocity.z = 0;
			// current_goal.yaw_rate = 0;
			// 可能需要让无人机悬停一段时间以确保它稳定下来
			// ros::Duration(1.0).sleep();
			ROS_INFO("起飞并自旋完成");
			// local_pos_pub.publish(current_goal);
		}

		// if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
		if (receive) // 触发后进行轨迹跟踪
		{

			// ROS_WARN("ego.position_x = %.2f,pos_x = %.2f,ego.position_y = %.2f,pos_y = %.2f,ego.position_z = %.2f,pos_z = %.2f",
			// 		 ego.position.x, position_x, ego.position.y, position_y, ego.position.z, position_z);
			float yaw_erro;
			yaw_erro = (ego_yaw - current_yaw);
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 选择local系，一定要local系
			current_goal.header.frame_id = "world";
			current_goal.header.stamp = ros::Time::now();
			// current_goal.type_mask = 0b001000000111; // 设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
			// 设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这; // 这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
			current_goal.type_mask = 0b101111111011;
			current_goal.position.x = ego_pos_x;
			current_goal.position.y = ego_pos_y;
			current_goal.position.z = ego_pos_z;
			current_goal.velocity.x = (ego_pos_x - position_x) * 2;
			current_goal.velocity.y = (ego_pos_y - position_y) * 1.5;
			current_goal.velocity.z = (ego_pos_z - position_z) * 2;
			// current_goal.velocity.x = ego_vel_x;
			// current_goal.velocity.y = ego_vel_y;
			// current_goal.velocity.z = ego_vel_z;
			// current_goal.acceleration_or_force.x = ego_vel_x - vel_x;
			// current_goal.acceleration_or_force.y = ego_vel_y - vel_y;
			// current_goal.acceleration_or_force.z = ego_vel_z - vel_z;
			// current_goal.acceleration_or_force.x = ego_a_x;
			// current_goal.acceleration_or_force.y = ego_a_y;
			// current_goal.acceleration_or_force.z = ego_a_z;
			current_goal.yaw = ego_yaw;
			current_goal.yaw_rate = ego_yaw_rate;
			Eigen::Vector3d pos(ego_pos_x, ego_pos_y, ego_pos_z);
			Eigen::Vector3d vel(ego_vel_x, ego_vel_y, ego_vel_z);
			Eigen::Vector3d acc(ego_a_x, ego_a_y, ego_a_z);

			Desired_State des_state;
			des_state.pos = pos;
			des_state.vel = vel;
			des_state.acc = acc;
			des_state.yaw = ego_yaw;
			// posPID.set_desired_state(des_state);
			// posPID.set_current_state(state);
			// posPID.set_current_odom(odom);
			// Eigen::Vector4d u_att_from_pos_controller = posPID.update(50.0);
			// send_attitude_setpoint(u_att_from_pos_controller);
			// posPID.printf_result();
			Eigen::Vector3d att_des;
			att_des << 0, 0, ego_yaw;
			Eigen::Quaterniond q_des = quaternion_from_rpy(att_des);

			geometry_msgs::PoseStamped pos_pub;
			pos_pub.pose.position.x = ego_pos_x;
			pos_pub.pose.position.y = ego_pos_y;
			pos_pub.pose.position.z = ego_pos_z;
			pos_pub.pose.orientation.x = q_des.x();
			pos_pub.pose.orientation.y = q_des.y();
			pos_pub.pose.orientation.z = q_des.z();
			pos_pub.pose.orientation.w = q_des.w();
			px4_setpoint_position_pub.publish(pos_pub);
			// local_pos_pub.publish(current_goal);
			// ROS_INFO("EGO规划位置：pos_x = %.2f,pos_x = %.2f,pos_z = %.2f", current_goal.position.x, current_goal.position.y, current_goal.position.z);
			// ROS_INFO("EGO规划速度：vel_x = %.2f", sqrt(pow(current_goal.velocity.x, 2) + pow(current_goal.velocity.y, 2)));
		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
