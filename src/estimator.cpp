#include "estimator.h"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    // 读取参数
    nh.param("uav_id", uav_id, 0);
    // 定位数据输入源 0 for vicon, 2 for gazebo ground truth
    nh.param<int>("input_source", input_source, 0);
    //　定位设备偏移量
    nh.param<float>("offset_x", pos_offset[0], 0);
    nh.param<float>("offset_y", pos_offset[1], 0);
    nh.param<float>("offset_z", pos_offset[2], 0);
    nh.param<float>("offset_yaw", yaw_offset, 0);

    uav_name = "/uav" + std::to_string(uav_id);
    msg_name = uav_name + "/control";

    // Initialize
    init();

    // ================== Subscribers ================
    // Current state: from px4 via Mavros /plugins/sys_status.cpp
    state_sub = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, state_cb);

    // Current location (ENU). Note although px4 uses NED, mavros converts them into ENU
    // Current location from px4 via Mavros /plugins/local_position.cpp.
    // Corresponding Mavlink: LOCAL_POSITION_NED (#32), px4 uORB message: vehicle_local_position.msg
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, pos_cb);

    
    // Current velocity (ENU) from px4 via Mavros /plugins/local_position.cpp
    // Mavlink: LOCAL_POSITION_NED (#32), px4 uORB message: vehicle_local_position.msg
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, vel_cb);

    // Current Euler angle (ENU) from px4 via Mavros /plugins/imu.cpp读取)
    // Mavlink: ATTITUDE (#30), px4 uORB message vehicle_attitude.msg
    attitude_sub = nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data", 10, att_cb); 

    // // Mocap position
    // mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ uav_name + "/pose", 10, mocap_cb);

    // Gazebo simulated odometry
    gazebo_sub = nh.subscribe<nav_msgs::Odometry>(uav_name + "/prometheus/ground_truth", 10, gazebo_cb);

    
    // ======================= Publishers ========================
    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 100);

    // 【发布】无人机状态合集,包括位置\速度\姿态\模式等,供上层节点使用
    drone_state_pub = nh.advertise<prometheus_msgs::DroneState>(uav_name + "/prometheus/drone_state", 10);
    
    // 【发布】提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>(uav_name + "/prometheus/message/main", 10);

    
    // ======================= Timer =============================
    // publish vision based estimation (to px4). Currently from gazebo simulation
    ros::Timer timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision);

    // publish drone state (to px4). 
    // 定时器,发布 drone_state_pub,保证20Hz以上
    ros::Timer timer_drone_state_pub = nh.createTimer(ros::Duration(0.05), timercb_drone_state);

    // 频率
    ros::Rate rate(100.0);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}