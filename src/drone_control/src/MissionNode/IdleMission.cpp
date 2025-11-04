/**
 * @file IdleMission.cpp
 * @brief 空闲状态任务实现
 */
#include "MissionNode/IdleMission.h"
/**
 * @brief 构造函数
 * @param drone_control 无人机控制引用
 * @param nh ROS节点句柄
 * @param status_monitor 状态监控器引用
 * @param mavros_bridge MAVROS桥接器引用
 */
IdleMission::IdleMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge)
: MissionNode(drone_control, nh, status_monitor, mavros_bridge)
{
    // 注册启动飞行任务服务
    _start_mission_service = _nh.advertiseService("/start_flight", &IdleMission::start_mission_service_callback, this);
    // 空闲状态默认不执行飞行任务
    _is_for_fly=false;
}
/**
 * @brief 析构函数
 */
IdleMission::~IdleMission()=default;
/**
 * @brief 进入空闲状态时调用
 * 
 * 记录日志
 */
void IdleMission::on_enter()
{
    MissionNode::on_enter();
    _is_running = true;
}
/**
 * @brief 空闲状态更新方法
 * 
 * 保持无人机静止，不执行任何操作
 */
std::array<double,4> IdleMission::on_update()
{
    // 根据服务回调函数而不是该布尔值来判断是否切换到其他任务
    _is_finished = false;
    return MissionNode::on_update();
}
/**
 * @brief 退出空闲状态时调用
 * 
 * 目前为空实现
 */
void IdleMission::on_exit()
{
    // 退出空闲状态
    _is_running = false;
    // 重置GPS固定状态
    _status_monitor.reset_gps_fix();
}
/**
 * @brief 处理启动任务机服务请求
 * 
 * 当收到启动任务机服务请求时，切换到指定任务状态
 * 
 * @param req 服务请求
 * @param res 服务响应
 * @return 是否成功处理请求
 */
bool IdleMission::start_mission_service_callback(my_interfaces::Nav::Request& req, my_interfaces::Nav::Response& res)
{
    if (! _status_monitor.is_connected())
    {
        res.success = false;
        res.message = "Drone is not connected. Please check the connection.";
        ROS_WARN("Service call rejected: Drone is not connected.");
        return true; // 服务已处理（即使是拒绝）
    }
    
    if (! _is_running)
    {    
        res.success = false;
        res.message = "Idle mission is not running. A mission is already in progress.";
        ROS_WARN("Service call rejected: Idle mission is not active.");
        return true; // 服务已处理（即使是拒绝）
    }
    ROS_INFO("IdleMission: Received start mission request");
    if (req.test)
	{
        _drone_control.switch_mission("precision_land");
		// 启动任务
		res.success = true;
		res.message = "Precision land started.";
	}
	else
	{
        _drone_control.switch_mission("mission_nav");
		// 启动任务
		res.success = true;
		res.message = "Mission navigation started.";
	}
	
	return true;  // 返回true表示服务调用成功
}
