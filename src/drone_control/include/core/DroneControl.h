/**
 * @file DroneControl.h
 * @brief 无人机控制类
 * 
 * 该类负责管理无人机的状态监控、Mavros桥接、控制器池和服务调用。
 * 它提供了设置入口控制器、切换控制器和更新状态的方法。
 */
#pragma once
#include <ros/ros.h>
#include <unordered_map>

#include "core/StatusMonitor.h"
#include "core/MavrosBridge.h"
#include "core/NodeMachine.h"

/**
 * @class DroneControl
 * @brief 无人机控制类
 * 
 * 该类负责管理无人机的状态监控、Mavros桥接、控制器池和服务调用。
 * 它提供了设置入口控制器、切换控制器和更新状态的方法。
 */
class DroneControl
{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化无人机控制类，注册服务、状态监控和Mavros桥接。
     */
    DroneControl();
    /**
     * @brief 析构函数
     * 
     * 清理无人机控制类，释放资源。
     */
    ~DroneControl();

public:
    /**
     * @brief 进入当前任务
     * 
     * 调用当前任务的on_enter()方法，初始化任务。
     * NOTE: 该方法为空实现。
     */
    void on_enter(){}
    /**
     * @brief 更新当前任务的状态
     * 
     * 调用当前任务的on_update()方法，更新其状态。
     */
    void on_update();
    
    /**
     * @brief 切换当前任务
     * @param mission_name 目标任务名称
     */
    void switch_mission(const std::string& mission_name);

private:
    // 成员变量的实际初始化顺序是由它们在头文件 (.h) 中的声明顺序决定的
    ros::NodeHandle _nh;
    /**
     * @brief 状态监控器
     * 
     * 用于监控无人机的状态，如位置、速度、姿态等。
     */
    StatusMonitor _status_monitor;
    /**
     * @brief Mavros桥接器
     * 
     * 用于与MAVROS通信，发送和接收无人机的状态和命令。
     */
    MavrosBridge _mavros_bridge;
    /**
     * @brief 任务节点机
     * 
     * 用于管理和执行不同的任务节点，如导航、定位等。
     */
    NodeMachine _mission_machine;

private:
    void register_missions();
    /**
     * @brief 设置入口控制器的服务回调函数
     * @param req 服务请求，包含目标位置和测试标志
     * @param res 服务响应，设置执行结果和消息
     * @return 布尔值，表示服务调用是否成功
     */
};