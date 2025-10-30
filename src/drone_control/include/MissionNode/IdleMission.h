/**
 * @file IdleMission.h
 * @brief 空闲状态任务头文件
 * 负责在无人机未执行任何任务时监听启动任务机服务请求
 */
#pragma once

#include <my_interfaces/Nav.h>
#include "MissionNode/MissionNode.h"
/**
 * @class IdleMission
 * @brief 空闲状态任务类
 * 负责在无人机未执行任何任务时监听启动任务机服务请求
 */
class IdleMission : public MissionNode
{
public:
    IdleMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge);
    ~IdleMission();
    /**
     * @brief 进入空闲状态时调用
     * 
     * 记录日志
     */
    void on_enter() override;
    /**
     * @brief 空闲状态更新方法
     * 
     * 保持无人机静止，不执行任何操作
     */
    std::array<double,4> on_update() override;
    /**
     * @brief 退出空闲状态时调用
     * 
     * 目前为空实现
     */
    void on_exit() override;

private:
    /**
     * @brief 启动任务机服务
     */
    ros::ServiceServer _start_mission_service;
    /**
     * @brief 任务机是否正在运行
     */
    bool _is_running=false;
private:
    /**
     * @brief 处理启动任务机服务请求
     * 
     * 当收到启动任务机服务请求时，切换到指定任务状态
     * 
     * @param req 服务请求
     * @param res 服务响应
     * @return 是否成功处理请求
     */
    bool start_mission_service_callback(my_interfaces::Nav::Request& req, my_interfaces::Nav::Response& res);
};
