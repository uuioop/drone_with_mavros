/**
 * @file MissionNode.h
 * @brief 任务基类定义，所有具体任务的抽象接口
 */
#pragma once
#include <ros/ros.h>
#include <array>

#include "core/NodeBase.h"
#include "core/StatusMonitor.h"
#include "core/MavrosBridge.h"

#include "core/DroneControl.h"
/**
 * @class MissionNode
 * @brief 任务基类，定义了所有任务必须实现的接口和共享功能
 */
class MissionNode : public NodeBase
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄引用
     * @param status_monitor 状态监控器引用
     * @param mavros_bridge MAVROS桥接器引用
     */
    MissionNode(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge):
    _drone_control(drone_control),_nh(nh),_status_monitor(status_monitor),_mavros_bridge(mavros_bridge)
    {}
    
    /**
     * @brief 析构函数
     */
    ~MissionNode()=default;
public:
    /**
     * @brief 任务进入时调用的方法，用于初始化
     */
    void on_enter() override
    {
        // 任务开始时，默认任务未完成
        set_finished(false);
        // 切换到任务要求的飞行模式
        _mavros_bridge.switch_flight_mode(_required_flight_mode);
        // 如果任务需要飞行，且无人机未启动，则启动无人机
        if (!_status_monitor.is_armed() && _is_for_fly)
        {
            _mavros_bridge.arm();
        }
    }
    
    /**
     * @brief 任务更新方法，在主循环中被持续调用    
     * 
     * @return std::array<double,4> 默认返回零值数组(不使用返回值)
     */
    std::array<double,4> on_update() override
    {
        if (_is_finished)
            _drone_control.switch_mission(_next_mission_name);
        return {0.0,0.0,0.0,0.0};
    }
    
    /**
     * @brief 任务退出时调用的方法，用于清理资源
     */
    virtual void on_exit() override=0;
    
    /**
     * @brief 设置任务完成标志
     * @param flag 任务完成标志，为true时表示任务已完成
     * @note 给予原子状态可调用的函数，用于设置任务完成状态
     */
    void set_finished(bool flag)
    {
        _is_finished=flag;
    }
    
    /**
     * @brief 获取状态监控器引用
     * @return 状态监控器引用
     */
    StatusMonitor& get_status_monitor() const
    {
        return _status_monitor;
    }

protected:
    DroneControl& _drone_control;
    /**
     * @brief ROS节点句柄引用
     */
    ros::NodeHandle& _nh;
    
    /**
     * @brief 状态监控器引用，用于获取无人机状态信息
     */
    StatusMonitor& _status_monitor;
    
    /**
     * @brief MAVROS桥接器引用，用于与无人机通信
     */
    MavrosBridge& _mavros_bridge;

    /**
     * @brief 下一个任务名称，用于任务切换
     */
    std::string _next_mission_name;

    /**
     * @brief 任务要求的飞行模式，默认值为AUTO.LOITER
     */
    std::string _required_flight_mode="AUTO.LOITER";
    
    /**
     * @brief 任务更新频率，默认值为5Hz
     */
    ros::Rate _rate{5.0};

    /**
     * @brief 任务完成标志，为true时表示任务已完成
     */
    bool _is_finished = false;

    /**
     * @brief 任务是否用于飞行，默认值为true
     */
    bool _is_for_fly = true;
};