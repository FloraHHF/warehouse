#ifndef HOVER_TEST_H
#define HOVER_TEST_H

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "message_utils.h"
#include "uav_utils/geometry_utils.h"
#include "math_utils.h"
#include "printf_utils.h"

using namespace std;


#define NODE_NAME "hover_test"            // name of the node

// variables
int uav_id;										// UAV id
string uav_name;

prometheus_msgs::SwarmCommand Command_Now;      // current command
prometheus_msgs::SwarmCommand Command_Last;     // last command
string msg_name;								// message name
prometheus_msgs::Message message;           // message to be printed. 

bool flag_printf;							// print

int controller_flag;
int controller_hz;

float Takeoff_height;                           // takeoff height
Eigen::Vector3d Takeoff_position;               // takeoff position
Eigen::Vector3f gazebo_offset;                  // gazebo offset, for ekf2_gps

// subscribers
ros::Subscriber command_sub;
ros::Subscriber drone_state_sub;
ros::Subscriber position_target_sub;

// publishers
ros::Publisher setpoint_raw_local_pub;
ros::Publisher setpoint_raw_attitude_pub;
ros::Publisher message_pub;

// services
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

// drone state
Eigen::Vector3d pos_drone;                      // drone state: position
Eigen::Vector3d vel_drone;                      // drone state: velocity
Eigen::Quaterniond q_drone;                 	// drone state: quaternion
double yaw_drone;

// goal
prometheus_msgs::DroneState _DroneState;        // drone status
Eigen::Vector3d pos_des(0,0,0);         
Eigen::Vector3d vel_des(0,0,0);         
Eigen::Vector3d acc_des(0,0,0);    
Eigen::Vector3d throttle_sp(0,0,0);                      
double yaw_des;  


float int_start_error;
Eigen::Vector3d int_e_v;            	// integrated error
Eigen::Quaterniond u_q_des;   			// desired quaternion
Eigen::Vector4d u_att;                  // desired attitude (rad) and desired throttle (0-1)

// control parameters
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


void init(ros::NodeHandle &nh)
{

    // UAV ID
    nh.param<int>("uav_id", uav_id, 0);
    uav_name = "/uav" + std::to_string(uav_id);

    // Controller flag: 0 for attitude controller (this program), 1 for position/velocity control (PX4)
    nh.param<int>("controller_flag", controller_flag, 1);

    //
    nh.param<int>("controller_hz", controller_hz, 50);
    // nh.param<int>("collision_flag", collision_flag, 1);
    
    // takeoff height, disarm height, landing speed, land mode
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    // nh.param<float>("Disarm_height", Disarm_height, 0.2);
    // nh.param<float>("Land_speed", Land_speed, 0.2);
    // nh.param<int>("Land_mode", Land_mode, 0);


    // Whether to print
    nh.param<bool>("flag_printf", flag_printf, false);
    
    // Settings for ekf2_gps
    nh.param<float>("gazebo_offset_x", gazebo_offset[0], 0);
    nh.param<float>("gazebo_offset_y", gazebo_offset[1], 0);
    nh.param<float>("gazebo_offset_z", gazebo_offset[2], 0);

    // Controller parameters
    nh.param<float>("quad_mass" , quad_mass, 1.5f);
    nh.param<float>("hov_percent" , hov_percent, 0.47f);
    nh.param<float>("Limit/pxy_int_max"  , int_max[0], 5.0);
    nh.param<float>("Limit/pxy_int_max"  , int_max[1], 5.0);
    nh.param<float>("Limit/pz_int_max"   , int_max[2], 10.0);
    nh.param<double>("hover_gain/Kp_xy", Kp_hover(0,0), 2.0f);
    nh.param<double>("hover_gain/Kp_xy", Kp_hover(1,1), 2.0f);
    nh.param<double>("hover_gain/Kp_z" , Kp_hover(2,2), 2.0f);
    nh.param<double>("hover_gain/Kv_xy", Kv_hover(0,0), 2.0f);
    nh.param<double>("hover_gain/Kv_xy", Kv_hover(1,1), 2.0f);
    nh.param<double>("hover_gain/Kv_z" , Kv_hover(2,2), 2.0f);
    nh.param<double>("hover_gain/Kvi_xy", Kvi_hover(0,0), 0.3f);
    nh.param<double>("hover_gain/Kvi_xy", Kvi_hover(1,1), 0.3f);
    nh.param<double>("hover_gain/Kvi_z" , Kvi_hover(2,2), 0.3f);
    nh.param<double>("hover_gain/Ka_xy", Ka_hover(0,0), 1.0f);
    nh.param<double>("hover_gain/Ka_xy", Ka_hover(1,1), 1.0f);
    nh.param<double>("hover_gain/Ka_z" , Ka_hover(2,2), 1.0f);
    nh.param<float>("hover_gain/tilt_angle_max" , tilt_angle_max_hover, 10.0f);

    nh.param<double>("track_gain/Kp_xy", Kp_track(0,0), 3.0f);
    nh.param<double>("track_gain/Kp_xy", Kp_track(1,1), 3.0f);
    nh.param<double>("track_gain/Kp_z" , Kp_track(2,2), 3.0f);
    nh.param<double>("track_gain/Kv_xy", Kv_track(0,0), 3.0f);
    nh.param<double>("track_gain/Kv_xy", Kv_track(1,1), 3.0f);
    nh.param<double>("track_gain/Kv_z" , Kv_track(2,2), 3.0f);
    nh.param<double>("track_gain/Kvi_xy", Kvi_track(0,0), 0.1f);
    nh.param<double>("track_gain/Kvi_xy", Kvi_track(1,1), 0.1f);
    nh.param<double>("track_gain/Kvi_z" , Kvi_track(2,2), 0.1f);
    nh.param<double>("track_gain/Ka_xy", Ka_track(0,0), 1.0f);
    nh.param<double>("track_gain/Ka_xy", Ka_track(1,1), 1.0f);
    nh.param<double>("track_gain/Ka_z" , Ka_track(2,2), 1.0f);
    nh.param<float>("track_gain/tilt_angle_max" , tilt_angle_max_track, 20.0f);

    msg_name = uav_name + "/control";
    
    // initialize
    Command_Now.Mode                = prometheus_msgs::SwarmCommand::Idle;
    Command_Now.Command_ID          = 0;
    Command_Now.position_ref[0]     = 0;
    Command_Now.position_ref[1]     = 0;
    Command_Now.position_ref[2]     = 0;
    Command_Now.yaw_ref             = 0;

    int_start_error = 2.0;
    int_e_v.setZero();
    u_att.setZero();
    g_ << 0.0, 0.0, 9.8;


    //Flora: this is originally in swarm_command_cb. 
    // I deleted tracking control, therefore put hover gains here
    Kp = Kp_hover;
    Kv = Kv_hover;
    Kvi = Kvi_hover;
    Ka = Ka_hover;
    tilt_angle_max = tilt_angle_max_hover;

}


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void swarm_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg)
{
    // CommandID必须递增才会被记录
    Command_Now = *msg;
    cout << "Received command" << Command_Now.position_ref[0] << endl;

    
    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    // if(Command_Last.Mode == prometheus_msgs::SwarmCommand::Disarm)
    // {
    //     Command_Now = Command_Last;
    // }else
    // {
    //     Command_Now.Mode = prometheus_msgs::SwarmCommand::Position_Control;
    // }
    

}


void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    pos_drone  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_drone  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    q_drone.w() = msg->attitude_q.w;
    q_drone.x() = msg->attitude_q.x;
    q_drone.y() = msg->attitude_q.y;
    q_drone.z() = msg->attitude_q.z;    

    yaw_drone = uav_utils::get_yaw_from_quaternion(q_drone);
}


void idle()
{
    mavros_msgs::PositionTarget pos_setpoint;

    
    // For how does the onboard controller receive message, see mavlink_receiver.cpp
    // For how does the onboard controller implement the command, FlightTaskOffboard.cpp
    pos_setpoint.type_mask = 0x4000;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    pos_setpoint.position.z = 0;

    pos_setpoint.yaw = 0;


    setpoint_raw_local_pub.publish(pos_setpoint);
}


// send desired attitude to px4: quaternion (from roll pitch yaw) and thrust
void send_attitude_setpoint(Eigen::Vector4d& u_att)
{
    mavros_msgs::AttitudeTarget att_setpoint;
    //geometry_msgs/Quaternion

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 0b00000111;

    Eigen::Vector3d att_des;
    att_des << u_att(0), u_att(1), u_att(2);

    Eigen::Quaterniond q_des = quaternion_from_rpy(att_des);

    att_setpoint.orientation.x = q_des.x();
    att_setpoint.orientation.y = q_des.y();
    att_setpoint.orientation.z = q_des.z();
    att_setpoint.orientation.w = q_des.w();
    att_setpoint.thrust = u_att(3);

    setpoint_raw_attitude_pub.publish(att_setpoint);
}


// desired velocity and position 
void send_pos_vel_xyz_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 2. position setpoint + velocity setpoint (velocity used as feedforward)
    // 控制方法请见 PositionControl.cpp
    pos_setpoint.type_mask = 0b100111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

// desired velocity in x y, desired position in z, desired yaw
void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b100111000011;   // 100 111 000 011  vx vy vz z + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
    
    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

// send desired position to px4: x y z yaw
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}


/*  Input: drone   position, velocity, yaw 
		  desired position, velocity, yaw, acceleration
	Output: roll pitch yaw thottle (0 to 1)	 */
void pos_controller()
{
    // integral term for position control only
	if (vel_des(0) != 0.0 || vel_des(1) != 0.0 || vel_des(2) != 0.0) 
    {
		//ROS_INFO("Reset integration");
		int_e_v.setZero();
	}

    Eigen::Vector3d pos_error = pos_des - pos_drone;

    Eigen::Vector3d u_pos = Kp * pos_error;

    Eigen::Vector3d vel_error  = u_pos + vel_des - vel_drone;

    Eigen::Vector3d u_vel = Kv * vel_error;  

    Eigen::Vector3d u_int = Kvi* int_e_v;

    for (int i=0; i<3; i++)
    {
        // 只有在pos_error比较小时，才会启动积分
        if(abs(pos_error[i]) < int_start_error)
        {
            int_e_v[i] += pos_error[i] * 0.01;

            if(abs(int_e_v[i]) > int_max[i])
            {
                cout << YELLOW << "----> int_e_v saturation [ "<< i << " ]"<<" [int_max]: "<<int_max[i]<<" [m/s] "<< TAIL << endl;
                
                int_e_v[i] = (int_e_v[i] > 0) ? int_max[i] : -int_max[i];
            }
        }else
        {
            int_e_v[i] = 0;
        }

        // If not in OFFBOARD mode, set all intergral to zero.
        if(_DroneState.mode != "OFFBOARD")
        {
            int_e_v[i] = 0;
        }
    }

    Eigen::Vector3d u_v = u_vel + u_int;

	// desired force = mass*control input + gravity + desired acceleration*mass*Ka
    Eigen::Vector3d F_des;
	F_des = u_v * quad_mass + quad_mass * g_ + Ka * quad_mass * acc_des;
    
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * quad_mass * g_(2))
	{
		ROS_INFO("thrust too low");
		F_des = F_des / F_des(2) * (0.5 * quad_mass * g_(2));
	}
	else if (F_des(2) > 2 * quad_mass * g_(2))
	{
		ROS_INFO("thrust too high");
		F_des = F_des / F_des(2) * (2 * quad_mass * g_(2));
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(uav_utils::toRad(tilt_angle_max)))
	{
		// ROS_INFO("pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(uav_utils::toRad(tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(uav_utils::toRad(tilt_angle_max)))
	{
		// ROS_INFO("roll too tilt");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(uav_utils::toRad(tilt_angle_max));	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = uav_utils::rotz(yaw_drone);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    u_att(0)  = std::atan2(-fy, fz);
    u_att(1)  = std::atan2( fx, fz);
    u_att(2)  = yaw_des;

    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = q_drone.toRotationMatrix();
    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    // 机体系下的推力合力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr);
    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = quad_mass * g_(2) / hov_percent;

    // 油门 = 期望推力/最大推力
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    u_att(3) = u1 / full_thrust;

    if(u_att(3) < 0.1)
    {
        u_att(3) = 0.1;
        ROS_INFO("throttle too low");
    }

    if(u_att(3) > 1.0)
    {
        u_att(3) = 1.0;
        ROS_INFO("throttle too high");
    }
}


// print parameters
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> hover test Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "uav_name   : "<< uav_name <<endl;
    cout << "Takeoff_height   : "<< Takeoff_height<<" [m] "<<endl;
    // cout << "Disarm_height    : "<< Disarm_height <<" [m] "<<endl;
    // cout << "Land_speed       : "<< Land_speed <<" [m/s] "<<endl;
}


#endif