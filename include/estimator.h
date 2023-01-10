#ifndef ESTIMATOR_H
#define ESTIMATOR_H

// 头文件
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>

#include <prometheus_msgs/DroneState.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

#include "math_utils.h"
#include "message_utils.h"

// 宏定义
#define NODE_NAME "estimator"         // node name
#define TIMEOUT_MAX 0.1               // MOCAP timeout limit
// 变量
int uav_id;
string uav_name;                            // UAV id, used as prefix
string object_name;                         // 动作捕捉软件中设定的刚体名字
string msg_name;
int input_source;                           // 0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
Eigen::Vector3f pos_offset;                 // 定位设备偏移量
float yaw_offset;                           // 定位设备偏移量
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Vector3d pos_drone_mocap;            // 无人机当前位置 (mocap)
Eigen::Quaterniond q_mocap;                 // 无人机当前姿态 - 四元数 (mocap)
Eigen::Vector3d Euler_mocap;                // 无人机当前姿态 - 欧拉角 (mocap)
ros::Time mocap_timestamp;                  // mocap时间戳
Eigen::Vector3d pos_drone_gazebo;           // 无人机当前位置 (gazebo)
Eigen::Quaterniond q_gazebo;                // 无人机当前姿态 - 四元数 (gazebo)
Eigen::Vector3d Euler_gazebo;               // 无人机当前姿态 - 欧拉角 (gazebo)
prometheus_msgs::Message message;           // 待打印的文字消息
nav_msgs::Odometry Drone_odom;              // 无人机里程计,用于rviz显示
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;    // 无人机轨迹容器,用于rviz显示
// 订阅话题
ros::Subscriber state_sub;
ros::Subscriber extended_state_sub;
ros::Subscriber position_sub;
ros::Subscriber velocity_sub;
ros::Subscriber attitude_sub;
ros::Subscriber alt_sub;
ros::Subscriber mocap_sub;
ros::Subscriber gazebo_sub;
// 发布话题
ros::Publisher drone_state_pub;
ros::Publisher vision_pub;
ros::Publisher message_pub;

void init()
{
    _DroneState.connected = false;
    _DroneState.armed = false;
    _DroneState.mode = "";
    _DroneState.position[0] = 0.0;
    _DroneState.position[1] = 0.0;
    _DroneState.position[2] = 0.0;
    _DroneState.velocity[0] = 0.0;
    _DroneState.velocity[1] = 0.0;
    _DroneState.velocity[2] = 0.0;
    _DroneState.attitude_q.w = 0.0;
    _DroneState.attitude_q.x = 0.0;
    _DroneState.attitude_q.y = 0.0;
    _DroneState.attitude_q.z = 0.0;
    _DroneState.attitude[0] = 0.0;
    _DroneState.attitude[1] = 0.0;
    _DroneState.attitude[2] = 0.0;
    _DroneState.attitude_rate[0] = 0.0;
    _DroneState.attitude_rate[1] = 0.0;
    _DroneState.attitude_rate[2] = 0.0;
    _DroneState.rel_alt = 0.0;
    //
    pos_drone_mocap = Eigen::Vector3d(0.0, 0.0, 0.0);
    q_mocap         = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    Euler_mocap     = Eigen::Vector3d(0.0, 0.0, 0.0);

    pos_drone_gazebo = Eigen::Vector3d(0.0, 0.0, 0.0);
    q_gazebo         = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    Euler_gazebo     = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    _DroneState.connected = msg->connected;
    _DroneState.armed = msg->armed;
    _DroneState.mode = msg->mode;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _DroneState.position[0] = msg->pose.position.x;
    _DroneState.position[1] = msg->pose.position.y;
    _DroneState.position[2] = msg->pose.position.z;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    _DroneState.velocity[0] = msg->twist.linear.x;
    _DroneState.velocity[1] = msg->twist.linear.y;
    _DroneState.velocity[2] = msg->twist.linear.z;
}

void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    
    _DroneState.attitude_q.w = q_fcu.w();
    _DroneState.attitude_q.x = q_fcu.x();
    _DroneState.attitude_q.y = q_fcu.y();
    _DroneState.attitude_q.z = q_fcu.z();

    _DroneState.attitude[0] = euler_fcu[0];
    _DroneState.attitude[1] = euler_fcu[1];
    _DroneState.attitude[2] = euler_fcu[2];

    _DroneState.attitude_rate[0] = msg->angular_velocity.x;
    _DroneState.attitude_rate[1] = msg->angular_velocity.x;
    _DroneState.attitude_rate[2] = msg->angular_velocity.x;
}

// For indoor experiment in the future
// void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
//     Euler_mocap = quaternion_to_euler(q_mocap);
//     // 记录收到mocap的时间戳
//     mocap_timestamp = ros::Time::now();
// }

void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "world")
    {
        pos_drone_gazebo = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Euler_gazebo = quaternion_to_euler(q_gazebo);
    }
    else
    {
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "wrong gazebo ground truth frame id.");
    }
}

void timercb_vision(const ros::TimerEvent &e)
{
    // For indoor experiment: assign mocap or odometry (from slam) to vision
    geometry_msgs::PoseStamped vision;
    vision.pose.position.x = pos_drone_gazebo[0];
    vision.pose.position.y = pos_drone_gazebo[1];
    vision.pose.position.z = pos_drone_gazebo[2];

    vision.pose.orientation.x = q_gazebo.x();
    vision.pose.orientation.y = q_gazebo.y();
    vision.pose.orientation.z = q_gazebo.z();
    vision.pose.orientation.w = q_gazebo.w();

    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void timercb_drone_state(const ros::TimerEvent &e)
{
    _DroneState.header.stamp = ros::Time::now();
    drone_state_pub.publish(_DroneState);
}

#endif