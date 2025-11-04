/**
 * @file GPSNavMission.h
 * @brief GPS导航任务类定义
 * @details 实现基于GPS的航点任务导航功能
 */
#pragma once

#include <array>
#include <vector>
#include <mavros_msgs/Waypoint.h>

#include "MissionNode/MissionNode.h"

/**
 * @class GPSNavMission
 * @brief GPS导航任务
 * @details 继承自MissionNode基类，实现基于GPS的自动任务导航功能
 */
class GPSNavMission : public MissionNode
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄引用
     * @param status_monitor 状态监控器引用
     * @param mavros_bridge MAVROS桥接器引用
     */
    GPSNavMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge);
    
    /**
     * @brief 析构函数
     */
    ~GPSNavMission();

    /**
     * @brief 进入状态时执行
     * @note 初始化导航相关参数和设置
     */
    void on_enter() override;
    
    /**
     * @brief 状态更新时执行
     * @note 实现导航逻辑，控制无人机到达目标位置
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出状态时执行
     * @note 清理导航相关资源和状态
     */
    void on_exit() override;

    /**
     * @brief 更新目标位置
     * @param position 新的目标位置，格式为{X, Y, Z}
     */
    void set_target_position(double latitude_deg, double longitude_deg, double relative_altitude_m)
    {
        _target_position = {latitude_deg, longitude_deg, relative_altitude_m};
    }

private:
    /**
     * @brief 目标位置数组 [纬度(度), 经度(度), 相对高度(米)]
     */
    std::array<double, 3> _target_position;
    /**
     * @brief 任务航点列表
     */
    std::vector<mavros_msgs::Waypoint> waypoints;
    /**
     * @brief 任务航点位置列表，每个元素为{纬度(度), 经度(度), 相对高度(米)}
     */
    std::vector<std::array<double, 3>> _waypoint_positions;
private:
    /**
     * @brief 上传任务航点到无人机
     */
    void upload_mission();
};
