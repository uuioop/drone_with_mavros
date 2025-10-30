#pragma once

#include <memory>

#include "MissionNode/MissionNode.h"
#include "core/NodeMachine.h"
#include "Tag/License.h"
#include "Tag/LicenseProcessor.h"

/**
 * @file ConfirmLicenseMission.h
 * @brief 号牌确认任务头文件
 * 负责管理无人机对目标号牌的搜索、确认和重新定位过程
 */

/**
 * @class ConfirmLicenseMission
 * @brief 号牌确认任务类
 * 继承自MissionNode基类，实现无人机对目标号牌的识别和确认控制逻辑
 */
class ConfirmLicenseMission:public MissionNode
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param status_monitor 状态监控器引用
     * @param mavros_bridge MAVROS桥接器引用
     */
    ConfirmLicenseMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge);
    
    /**
     * @brief 析构函数
     */
    ~ConfirmLicenseMission();

public:
    /**
     * @brief 切换状态机状态
     * @param id 目标状态ID
     */
    void switch_node(const std::string& id)
    {
        _state_machine.switch_node(id);
    }
    
    /**
     * @brief 进入任务时调用
     */
    void on_enter() override;
    
    /**
     * @brief 更新任务状态时调用
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出任务时调用
     */
    void on_exit() override;

    /**
     * @brief 获取号牌处理器
     * @return 号牌处理器智能指针
     */
    std::shared_ptr<LicenseProcessor> get_license_processor() const
    {
        return _license_processor;
    }
    
    /**
     * @brief 获取移动向量
     * @return 移动向量，包含方向和距离
     */
    std::tuple<char, double> get_move_vector() const
    {
        return _move_vector;
    }
    
    /**
     * @brief 更新移动向量
     * @param move_vector 新的移动向量
     */
    void update_move_vector(std::tuple<char, double> move_vector)
    {
        _move_vector = move_vector;
    }

    /**
     * @brief 更新目标号牌号码
     * @param license_no 新的目标号牌号码
     */
    void update_target_license_no(std::string license_no)
    {
        _license_processor->set_target_license(license_no);
    }

private:
    /**
     * @brief 状态机，管理搜索、确认和重新定位三种状态
     * 负责协调不同状态之间的转换和执行
     */
    NodeMachine _state_machine;
    
    /**
     * @brief 号牌配置信息
     * 存储号牌识别和处理相关的配置参数
     */
    LicenseConfig _license_config;
    
    /**
     * @brief 号牌处理器，用于处理和识别号牌信息
     * 封装了号牌检测、验证和比对的核心功能
     */
    std::shared_ptr<LicenseProcessor> _license_processor;
    
    /**
     * @brief 移动向量，默认向下移动1.0单位
     * 第一个元素为方向字符（'y'/'z'），第二个元素为移动距离和方向
     */
    std::tuple<char, double> _move_vector = {'z', -1.0};

private:
    /**
     * @brief 注册状态机中的各个状态
     * 初始化并注册搜索、确认和重新定位三种状态到状态机中
     */
    void register_states();
};
