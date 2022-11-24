#ifndef HOVER_TEST_H
#define HOVER_TEST_H

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "message_utils.h"
#include "uav_utils/geometry_utils.h"
#include "math_utils.h"
#include "printf_utils.h"

using namespace std;



// prometheus_msgs::SwarmCommand Command_Now;      // 无人机当前执行命令
// prometheus_msgs::SwarmCommand Command_Last;     // 无人机上一条执行命令


float Takeoff_height;                           // 默认起飞高度
Eigen::Vector3d Takeoff_position;               // 起飞位置
float Disarm_height;                            // 自动上锁高度
float Land_speed;                               // 降落速度
int Land_mode;                                  //降落模式
Eigen::Vector3f gazebo_offset;                  // 偏移量

// 订阅
ros::Subscriber command_sub;
ros::Subscriber drone_state_sub;
ros::Subscriber position_target_sub;
ros::Subscriber nei_state_sub[MAX_UAV_NUM+1];

// 发布
ros::Publisher setpoint_raw_local_pub;
ros::Publisher setpoint_raw_attitude_pub;
ros::Publisher message_pub;

// 服务
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

// 邻居状态量
prometheus_msgs::DroneState state_nei[MAX_UAV_NUM+1];
Eigen::Vector3d pos_nei[MAX_UAV_NUM+1];                     // 邻居位置
Eigen::Vector3d vel_nei[MAX_UAV_NUM+1];                     // 邻居速度
Eigen::Vector3d dv;
float R = 4.0;
float r = 0.5;
// 无人机状态量
Eigen::Vector3d pos_drone;                      // 无人机位置
Eigen::Vector3d vel_drone;                      // 无人机速度
Eigen::Quaterniond q_drone;                 // 无人机四元数
double yaw_drone;

// 目标设定值
prometheus_msgs::DroneState _DroneState;        // 无人机状态
Eigen::Vector3d pos_des(0,0,0);         
Eigen::Vector3d vel_des(0,0,0);         
Eigen::Vector3d acc_des(0,0,0);    
Eigen::Vector3d throttle_sp(0,0,0);                      
double yaw_des;  

float int_start_error;
Eigen::Vector3d int_e_v;            // 积分
Eigen::Quaterniond u_q_des;   // 期望姿态角（四元数）
Eigen::Vector4d u_att;                  // 期望姿态角（rad）+期望油门（0-1）

// 控制参数
Eigen::Matrix3d Kp_hover;
Eigen::Matrix3d Kv_hover;
Eigen::Matrix3d Kvi_hover;
Eigen::Matrix3d Ka_hover;
float tilt_angle_max_hover;
Eigen::Matrix3d Kp_track;
Eigen::Matrix3d Kv_track;
Eigen::Matrix3d Kvi_track;
Eigen::Matrix3d Ka_track;
float tilt_angle_max_track;
Eigen::Matrix3d Kp;
Eigen::Matrix3d Kv;
Eigen::Matrix3d Kvi;
Eigen::Matrix3d Ka;
float tilt_angle_max;
Eigen::Vector3d g_;
float quad_mass;
float hov_percent;      // 0-1
Eigen::Vector3f int_max;

// 编队控制相关
Eigen::MatrixXf formation_separation;           // 阵型偏移量
float k_p;                                      // 速度控制参数
float k_aij;                                    // 速度控制参数
float k_gamma;                                  // 速度控制参数
float yita;                                     // 速度控制参数