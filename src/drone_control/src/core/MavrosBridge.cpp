/**
 * @file MavrosBridge.cpp
 * @brief MavrosBridge类的实现文件，提供与MAVROS通信的具体实现
 */
#include <cmath>  // 用于 M_PI 常量

#include "core/MavrosBridge.h"
#include "util/utils.h"

/**
 * @brief MavrosBridge构造函数
 * @param nh ROS节点句柄引用
 * @param status_monitor 状态监控器引用
 * @note 初始化服务客户端和发布者
 */
MavrosBridge::MavrosBridge(ros::NodeHandle& nh, StatusMonitor& status_monitor)
    : _nh(nh), _status_monitor(status_monitor)
{
    _arm_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    _local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    _global_pos_pub = _nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    _body_vel_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
}

/**
 * @brief MavrosBridge析构函数
 * @note 使用默认实现
 */
MavrosBridge::~MavrosBridge()=default;

/**
 * @brief 连接到飞控单元(FCU)
 * @return 连接成功返回true
 * @note 等待与FCU建立连接
 */
bool MavrosBridge::connect_to_fcu()
{
    while(ros::ok() && !_status_monitor.is_connected()){
        ROS_INFO_ONCE("等待FCU连接...");
        _rate.sleep();
    }
    ROS_INFO("FCU is already connected");
    return true;
}

/**
 * @brief 设置无人机解锁或上锁状态
 * @param flag true为解锁，false为锁定
 * @return 设置成功返回true
 * @note 持续尝试直到状态正确设置
 */
bool MavrosBridge::set_arm(bool flag)
{
    if (_status_monitor.is_armed() == flag)
    {
        ROS_WARN("Drone is already %s", flag ? "armed" : "disarmed");
        return true;
    }
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = flag;
    while(ros::ok() && _status_monitor.is_armed() != flag){
        callService<mavros_msgs::CommandBool>(_arm_client, arm_srv, "/mavros/cmd/arming");
        _rate.sleep();
    }
    ROS_INFO("Drone is now %s", flag ? "armed" : "disarmed");
    return true;
}

/**
 * @brief 设置无人机飞行模式
 * @param mode 飞行模式名称
 * @return 设置成功返回true
 * @note 调用MAVROS的set_mode服务
 */
bool MavrosBridge::set_mode(const std::string& mode)
{
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = mode;
    return callService<mavros_msgs::SetMode>(_set_mode_client, set_mode_srv, "/mavros/set_mode");
}

/**
 * @brief 启动离板控制模式(OFFBOARD)
 * @return 启动成功返回true
 * @note 进入OFFBOARD模式前先发送100个悬停指令
 */
bool MavrosBridge::start_offboard_mode()
{
    // 在进入Offboard模式前，必须先发送一些设定点或速度指令（至少100个）
    for(int i = 100; ros::ok() && i > 0; --i){
        set_velocity_body({0.0, 0.0, 0.0, 0.0});
        _rate.sleep();
    }
    return set_mode("OFFBOARD");
}

/**
 * @brief 解锁无人机
 * @return 解锁成功返回true
 * @note 调用set_arm(true)实现
 */
bool MavrosBridge::arm()
{
    return set_arm(true);
}

/**
 * @brief 锁定无人机
 * @return 锁定成功返回true
 * @note 调用set_arm(false)实现
 */
bool MavrosBridge::disarm()
{
    return set_arm(false);
}

/**
 * @brief 切换无人机飞行模式
 * @param target_mode 目标飞行模式名称
 * @return 切换成功返回true
 * @note 如果目标模式为OFFBOARD，则需要特定的启动方法start_offboard_mode()
 * @note 否则直接尝试切换模式
 */
bool MavrosBridge::switch_flight_mode(const std::string& target_mode)
{
    auto current_mode = _status_monitor.get_flight_mode();
	if (current_mode == target_mode)
	{
		ROS_INFO("[DroneControl] Flight mode %s is already set.", target_mode.c_str());
		return true;
	}
	if (target_mode == "OFFBOARD")
		return start_offboard_mode();
	else
		// 非OFFBOARD模式切换，直接尝试切换
		return set_mode(target_mode);
}

/**
 * @brief 设置本地坐标系下的目标位置和姿态
 * @param pos 位置和偏航角 {x, y, z, yaw}
 * @param orient 方向四元数 {qx, qy, qz, qw}
 * @return 设置成功返回true
 * @note 使用ENU坐标系
 * @note 如果未指定orient参数，则使用pos中的yaw值计算四元数
 */
bool MavrosBridge::set_position_local(const std::array<double, 4>& pos,const std::array<double, 4>& orient)
{
    // 本地坐标系 MAVROS 使用 ENU (East-North-Up)
    geometry_msgs::PoseStamped local_pos_msg;

    local_pos_msg.pose.position.x = pos[0];
    local_pos_msg.pose.position.y = pos[1];
    local_pos_msg.pose.position.z = pos[2];
    // 若第二个参数为默认值，使用第一个参数的偏航角
    if(orient[0]==0.0 && orient[1]==0.0 && orient[2]==0.0 && orient[3]==1.0)
    {
        // 将偏航角转换为四元数
        double yaw_deg = pos[3]; // pos[3] 表示偏航角（度）
        double yaw_rad = deg2rad_normalized(yaw_deg); // 使用工具函数转换为弧度
        local_pos_msg.pose.orientation.x = 0.0;
        local_pos_msg.pose.orientation.y = 0.0;
        local_pos_msg.pose.orientation.z = sin(yaw_rad / 2.0);
        local_pos_msg.pose.orientation.w = cos(yaw_rad / 2.0);
    }
    else
    {
        local_pos_msg.pose.orientation.x = orient[0];
        local_pos_msg.pose.orientation.y = orient[1];
        local_pos_msg.pose.orientation.z = orient[2];
        local_pos_msg.pose.orientation.w = orient[3];
    }

    return publishMessage(_local_pos_pub, local_pos_msg, "odom");
}

/**
 * @brief 设置机体坐标系下的速度指令
 * @param vel 线速度和偏航角速率 {vx, vy, vz, yaw_rate}
 * @return 设置成功返回true
 * @note 使用FLU坐标系 (Forward-Left-Up)
 * @note MAVROS会自动处理ROS到PX4的坐标系转换
 */
bool MavrosBridge::set_velocity_body(const std::array<double, 4>& vel)
{
    // ROS_INFO("set_velocity_body: %f, %f, %f, %f", vel[0], vel[1], vel[2], vel[3]);
    mavros_msgs::PositionTarget pos_target_msg;
    // 坐标系设为机体坐标系
    // MAVROS 会处理 ROS(FLU) 到 PX4(FRD) 的坐标转换
    pos_target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

    // 使用 type_mask 来指定哪些字段是有效的
    // 我们要控制速度和偏航角速率，所以忽略位置、加速度和偏航角本身
    pos_target_msg.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW;

    // 设置线速度 (遵循 FLU: Forward, Left, Up 约定)
    pos_target_msg.velocity.x = vel[0];
    pos_target_msg.velocity.y = vel[1];
    pos_target_msg.velocity.z = vel[2];
    // PositionTarget 只能控制偏航角速率，无法直接控制翻滚和俯仰角速率
    pos_target_msg.yaw_rate = deg2rad_normalized(vel[3]);
    
    return publishMessage(_body_vel_pub, pos_target_msg, "base_link");
}

