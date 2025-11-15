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
    _next_mission_name="precision_land";
    _rate = 1;
}

/**
 * @brief 析构函数实现
 * 
 * 使用默认析构函数，自动管理资源释放
 */
GPSNavMission::~GPSNavMission()=default;

void GPSNavMission::upload_mission()
{
    auto global_pos = _status_monitor.get_global_pos();
    _waypoint_positions = {
        {global_pos[0], global_pos[1], 10}
    };
    // 发送航点位置到无人机
    _mavros_bridge.upload_mission(_waypoint_positions);
}

/**
 * @brief 进入GPS导航任务时的回调实现
 * 
 * 目前为空实现，可根据需要添加初始化逻辑
 */
void GPSNavMission::on_enter()
{
    auto entry_time = ros::Time::now();
    while(ros::ok() && !_status_monitor.is_gps_fixed())
    {
        ROS_WARN("Waiting for GPS fix...");
        if (ros::Time::now() - entry_time > ros::Duration(10.0))
        {
            _drone_control.switch_mission("idle");
            ROS_ERROR("Failed to get GPS fix after 10 seconds. Back to idle mode.");
            return;
        }
        _rate.sleep();
    }
    // 初始化航点位置并发送
    upload_mission();
    
    MissionNode::on_enter();
}

/**
 * @brief 更新GPS导航任务状态时的回调实现
 * 
 * 目前为空实现，可根据需要添加GPS导航逻辑
 */
std::array<double,4> GPSNavMission::on_update() 
{
    // 根据服务回调函数而不是该布尔值来判断是否切换到其他任务
    _is_finished = _status_monitor.is_mission_completed() || _status_monitor.is_mission_aborted();
    // 后续再处理，先默认切换到同一个任务
    if (_status_monitor.is_mission_aborted()) ;
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
