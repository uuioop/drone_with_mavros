/**
 * @file GPSNavMission.cpp
 * @brief 任务导航任务节点实现文件
 * 
 * 该文件实现了基于任务的导航任务节点，用于无人机按照预设任务点进行导航。
 */
#include "MissionNode/GPSNavMission.h"

/**
 * @brief 构造函数实现
 * 
 * 初始化GPS导航任务节点
 * 
 * @param nh ROS节点句柄
 * @param status_monitor 状态监控器引用
 * @param mavros_bridge MAVROS桥接器引用
 */
GPSNavMission::GPSNavMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge)
    : MissionNode(drone_control, nh, status_monitor, mavros_bridge)
{
    _required_flight_mode ="AUTO.MISSION";
    _next_mission_name="confirm_license";
}

/**
 * @brief 析构函数实现
 * 
 * 使用默认析构函数，自动管理资源释放
 */
GPSNavMission::~GPSNavMission()=default;

/**
 * @brief 进入GPS导航任务时的回调实现
 * 
 * 目前为空实现，可根据需要添加初始化逻辑
 */
void GPSNavMission::on_enter()
{
    // 初始化任务点
    // _mission_machine.set_mission_points(_mission_points);
    MissionNode::on_enter();
}

/**
 * @brief 更新GPS导航任务状态时的回调实现
 * 
 * 目前为空实现，可根据需要添加GPS导航逻辑
 */
std::array<double,4> GPSNavMission::on_update() 
{
    return MissionNode::on_update();
}

/**
 * @brief 退出GPS导航任务时的回调实现
 * 
 * 目前为空实现，可根据需要添加清理逻辑
 */
void GPSNavMission::on_exit()
{
}
