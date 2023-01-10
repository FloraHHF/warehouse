/*
The goal of this code is to implement ORCA on P450 drones
The code is adapted from swarm_controller
The following code are removed from swarm_controller for simplicity:
1. code regarding swarm control
2. code regarding debug
3. code regarding geo_fence

TODO: change prometheus_msgs::SwarmCommand to prometheus_msgs::ControlCommand

*/



#include "hover_test.h"

using namespace std;
void mainloop_cb(const ros::TimerEvent &e);
void control_cb(const ros::TimerEvent &e);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> main <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    // Initialize and read parameters
    init(nh);

    
    // Suggested rate: 10~50Hz. Use higher rate for velocity/acceleration control
    ros::Rate rate(controller_hz);

    
    // Print parameters
    if(flag_printf)
    {
        printf_param();
    }

    // //【订阅】集群控制指令
    command_sub = nh.subscribe<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 10, swarm_command_cb);

    // Subscriber to drone status
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10, drone_state_cb);


    // Publisher: desired position/velocity/acceleration in ENU frame
    // To PX4 through Mavros (/plugins/setpoint_raw.cpp)
    // using Mavlink message SET_POSITION_TARGET_LOCAL_NED
    // corresponding uORB message position_setpoint_triplet.msg
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);
    
    // Publisher: desired attitude
    setpoint_raw_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(uav_name +  "/mavros/setpoint_raw/attitude", 10);

    // Publisher: to ground station
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    // Service: arm/disarm
    // Through Mavros /plugins/command.cpp
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");

    
    // Service: change mode
    // Through Mavros /plugins/command.cpp
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");


    ros::Timer mainloop_timer = nh.createTimer(ros::Duration(0.1), mainloop_cb);
    ros::Timer control_timer = nh.createTimer(ros::Duration(1.0/controller_hz), control_cb);

    ros::spin();

    return 0;
}

void mainloop_cb(const ros::TimerEvent &e)
{

    switch (Command_Now.Mode)
    {
    // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
    case prometheus_msgs::SwarmCommand::Idle:
        
        // idle();

        // 设定yaw_ref=999时，切换offboard模式，并解锁
        if(Command_Now.yaw_ref == 999)
        {
            if(_DroneState.mode != "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "OFFBOARD";
                set_mode_client.call(mode_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Setting to OFFBOARD Mode...");
            }

            if(!_DroneState.armed)
            {
                arm_cmd.request.value = true;
                arming_client.call(arm_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, msg_name, "Arming...");
            }
        }
        break;

    // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度    
    case prometheus_msgs::SwarmCommand::Takeoff:
        
        // 设定起飞点
        if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Takeoff)
        {
            // 设定起飞位置
            Takeoff_position = pos_drone;
            pos_des[0] = pos_drone[0];
            pos_des[1] = pos_drone[1];
            pos_des[2] = pos_drone[2] + Takeoff_height;
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des    = yaw_drone;
        }

        // cout << "From hover_test.cpp, takeoff " << endl;
        break;

    // [Hold] hover at current location
    case prometheus_msgs::SwarmCommand::Hold:

        if (Command_Last.Mode != prometheus_msgs::SwarmCommand::Hold)
        {
            pos_des = pos_drone;
            vel_des << 0.0, 0.0, 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = yaw_drone;
        }
        break;

    // [Land] 
    case prometheus_msgs::SwarmCommand::Land:
        mode_cmd.request.custom_mode = "AUTO.LAND";
        set_mode_client.call(mode_cmd); 
        
        break;

    // [Disarm]
    case prometheus_msgs::SwarmCommand::Disarm:
        ROS_INFO_STREAM_ONCE ("---->Disarm....");
        if(_DroneState.mode == "OFFBOARD")
        {
            mode_cmd.request.custom_mode = "MANUAL";
            set_mode_client.call(mode_cmd);
        }

        if(_DroneState.armed)
        {
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
        }
        break;

    case prometheus_msgs::SwarmCommand::Position_Control:

        //　此控制方式即为　集中式控制，　直接由地面站指定期望位置点
        pos_des[0] = Command_Now.position_ref[0] - gazebo_offset[0];
        pos_des[1] = Command_Now.position_ref[1] - gazebo_offset[1];
        pos_des[2] = Command_Now.position_ref[2] - gazebo_offset[2];
        yaw_des = Command_Now.yaw_ref;
        break;

    case prometheus_msgs::SwarmCommand::Velocity_Control:

        // TODO: ORCA
        break;

    case prometheus_msgs::SwarmCommand::Move:

        pos_des[0] = Command_Now.position_ref[0];
        pos_des[1] = Command_Now.position_ref[1];
        pos_des[2] = Command_Now.position_ref[2];
        vel_des << 0.0, 0.0, 0.0;
        acc_des << 0.0, 0.0, 0.0;
        yaw_des = Command_Now.yaw_ref;
        cout << RED  << "Wrong swarm command: switch to XYZ_POS "  << TAIL << endl;

        break;

    case prometheus_msgs::SwarmCommand::User_Mode1:

        break;
    }
    Command_Last = Command_Now;
}

void control_cb(const ros::TimerEvent &e)
{
    if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Idle  ||
         Command_Now.Mode == prometheus_msgs::SwarmCommand::Disarm  )
    {
        return;
    }

    if(controller_flag == 0)
    {
        // 计算控制量
        pos_controller();

        // 发送角度期望值
        send_attitude_setpoint(u_att);
    }else
    {
        if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Takeoff )
        {
            send_pos_setpoint(pos_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Hold )
        {
            send_pos_setpoint(pos_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Land )
        {
            send_pos_vel_xyz_setpoint(pos_des, vel_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Position_Control )
        {
            send_pos_setpoint(pos_des, yaw_des);
        }else if( Command_Now.Mode == prometheus_msgs::SwarmCommand::Velocity_Control )
        {
            send_vel_xy_pos_z_setpoint(pos_des, vel_des, yaw_des);
        }else
        {
            cout << RED  << "Wrong swarm command!"  << TAIL << endl;
        }      
    }

}
