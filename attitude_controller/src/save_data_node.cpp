
#include <iostream>
#include <time.h>
#include <ros/ros.h> 
#include <tf/transform_datatypes.h>
#include <fstream>  //文件流库函数
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
// #include <quadrotor_msgs/FlatTarget.h>

double pi = 3.1415926;

using namespace std;

ofstream outfile_vrpnPose;   //输出流
ofstream outfile_t265Pose;   //输出流
ofstream outfile_px4Pose;   //输出流
ofstream outfile_targetPose;   //输出流
ofstream outfile_targetYaw;   //期望偏航角
ofstream outfile_px4Velocity;   //期望偏航角

geometry_msgs::PoseStamped vrpnPose;
geometry_msgs::PoseStamped realsenseBridgePose;
geometry_msgs::TwistStamped px4Velocity;

geometry_msgs::PoseStamped px4Pose;
// quadrotor_msgs::FlatTarget targetPose;

bool targetPose_rec_flag = false;
bool px4Pose_rec_flag = false;
bool px4Velocity_rec_flag = false;

void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
// void targetPose_cb(const quadrotor_msgs::FlatTarget::ConstPtr& msg);
void px4Velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);


int main(int argc, char *argv[])
{
    
    //视频保存位置
    string saveFilePath = "/home/linux/work/dataset/graduation/";  

    ros::init(argc, argv, "save_data_node");
    ros::NodeHandle nh;//创建句柄
        
    ros::Subscriber vrpn_pose_sub;
    ros::Subscriber realsense_bridge_pose_sub;
    ros::Subscriber px4Velocity_sub;

    ros::Subscriber px4Pose_sub;
    ros::Subscriber targetPose_sub;
    ros::Subscriber targetYaw_sub;

    //设置订阅主题 
    // vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/px4_q250/pose", 1, vrpn_pose_cb);
    vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1, vrpn_pose_cb);
    realsense_bridge_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/px4/vision_pose/pose", 1, realsense_bridge_pose_cb);
    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,px4Pose_cb);
    // targetPose_sub = nh.subscribe<quadrotor_msgs::FlatTarget>("reference/flatsetpoint", 10,targetPose_cb);
    px4Velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, px4Velocity_cb);

    ros::Rate loop_rate(25);

    // 获取当前时间，精确到秒
    time_t currentTime = std::time(NULL);
    char chCurrentTime[64];
    std::strftime(chCurrentTime, sizeof(chCurrentTime), "%Y-%m-%d-%H-%M-%S", std::localtime(&currentTime)); //年月日 时分秒
    std::string stCurrentTime = chCurrentTime;// 转为string
    
    // 文件

    // outfile_vrpnPose.open(saveFilePath + "vrpnPose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);
    // outfile_t265Pose.open(saveFilePath + "t265Pose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);
    outfile_px4Pose.open(saveFilePath + "px4Pose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);
    outfile_targetPose.open(saveFilePath + "targetPose" + "-"  + stCurrentTime+ ".txt" , ios::trunc);
    outfile_px4Velocity.open(saveFilePath + "px4Velocity" + "-"  + stCurrentTime+ ".txt" , ios::trunc);

    if(!outfile_px4Velocity.is_open() || !outfile_px4Pose.is_open()  || !outfile_targetPose.is_open())
    {
        cout << "fail to open!" << endl;
        return -1;
    }

    double last_time =ros::Time::now().toSec();
    int time = 0;
    while(ros::ok()) 
    {

        // if(!px4Pose_rec_flag){
        //     cout << "px4Pose_rec_flag is flase " << endl;
        //     ros::spinOnce();
        //     loop_rate.sleep();
        //     continue;
        // }
        // if(!px4Velocity_rec_flag){
        //     cout << "px4Velocity_rec_flag is flase " << endl;
        //     ros::spinOnce();
        //     loop_rate.sleep();
        //     continue;
        // }
        if(!targetPose_rec_flag){
            cout << "targetPose_rec_flag is flase " << endl;
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        double secs =ros::Time::now().toNSec()/1000000000.0;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(px4Pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
        outfile_px4Pose << setiosflags(ios::fixed) << setprecision(7) 
                        << secs << "\t"
                        << px4Pose.pose.position.x << "\t" << px4Pose.pose.position.y << "\t" << px4Pose.pose.position.z << "\t"
                        << roll << "\t" << pitch << "\t" << yaw
                        << endl;

        outfile_px4Velocity << setiosflags(ios::fixed) << setprecision(7) 
                        << secs << "\t"
                        << px4Velocity.twist.linear.x << "\t" << px4Velocity.twist.linear.y << "\t" << px4Velocity.twist.linear.z << "\t"
                        << px4Velocity.twist.angular.x << "\t" << px4Velocity.twist.angular.y << "\t" << px4Velocity.twist.angular.z << "\t"
                        << endl;

        // outfile_targetPose << setiosflags(ios::fixed) << setprecision(7) 
        //                 << secs << "\t"
        //                 << targetPose.position.x << "\t" << targetPose.position.y << "\t" << targetPose.position.z << "\t"
        //                 << targetPose.yaw << "\t" << targetPose.yaw_dot << "\t"
        //                 << targetPose.velocity.x << "\t" << targetPose.velocity.y << "\t" << targetPose.velocity.z << "\t"
        //                 // << targetPose.acceleration.x << "\t" << targetPose.acceleration.y << "\t" << targetPose.acceleration.z << "\t"
        //                 << endl;

        px4Pose_rec_flag = false;
        px4Velocity_rec_flag = false;
        targetPose_rec_flag = false;

        if(ros::Time::now().toSec() - last_time >=2)
        {   time = time + 2;
            cout << "sava_data_node is running! time: " << time << endl;
            last_time =ros::Time::now().toSec();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/*通过vrpn接受bebop位置消息*/
void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    roll = roll * 180 / pi;
    pitch = pitch * 180 / pi;
    yaw = yaw * 180 / pi;

    double secs =ros::Time::now().toNSec()/1000000000.0;
    outfile_vrpnPose << setiosflags(ios::fixed) << setprecision(7) 
                     << secs << "\t"
                     << msg->pose.position.x << "\t" << msg->pose.position.y << "\t" << msg->pose.position.z << "\t"
                     << roll << "\t" << pitch << "\t" << yaw
                     << endl;
}

void realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    roll = roll * 180 / pi;
    pitch = pitch * 180 / pi;
    yaw = yaw * 180 / pi;

    double secs =ros::Time::now().toNSec()/1000000000.0;
    outfile_t265Pose << setiosflags(ios::fixed) << setprecision(7) 
                     << secs << "\t"
                     << msg->pose.position.x << "\t" << msg->pose.position.y << "\t" << msg->pose.position.z << "\t"
                     << roll << "\t" << pitch << "\t" << yaw
                     << endl;
}
void px4Velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    px4Velocity.twist = msg->twist;
    px4Velocity_rec_flag = true;
    // double secs =ros::Time::now().toNSec()/1000000000.0;
    // outfile_px4Velocity << setiosflags(ios::fixed) << setprecision(7) 
    //                 << secs << "\t"
    //                 << msg->twist.linear.x << "\t" << msg->twist.linear.y << "\t" << msg->twist.linear.z << "\t"
    //                 << msg->twist.angular.x << "\t" << msg->twist.angular.y << "\t" << msg->twist.angular.z << "\t"
    //                 << endl;
}
void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    px4Pose.pose = msg->pose;
    px4Pose_rec_flag = true;
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(msg->pose.orientation, quat);
    // double roll, pitch, yaw;
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // roll = roll * 180 / pi;
    // pitch = pitch * 180 / pi;
    // yaw = yaw * 180 / pi;
    
    // double secs =ros::Time::now().toNSec()/1000000000.0;
    // outfile_px4Pose << setiosflags(ios::fixed) << setprecision(7) 
    //                 << secs << "\t"
    //                 << msg->pose.position.x << "\t" << msg->pose.position.y << "\t" << msg->pose.position.z << "\t"
    //                 << roll << "\t" << pitch << "\t" << yaw
    //                 << endl;
}
// void targetPose_cb(const quadrotor_msgs::FlatTarget::ConstPtr& msg)
// {

//     targetPose = *msg;
//     targetPose_rec_flag = true;
//     // double secs =ros::Time::now().toNSec()/1000000000.0;
//     // outfile_targetPose << setiosflags(ios::fixed) << setprecision(7) 
//     //                 << secs << "\t"
//     //                 << msg->position.x << "\t" << msg->position.y << "\t" << msg->position.z << "\t"
//     //                 << msg->yaw << "\t" << msg->yaw_dot
//     //                 << endl;
// }
