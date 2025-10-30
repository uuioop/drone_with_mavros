#pragma once

#include <memory>
#include <ros/ros.h>

#include "StateNode/StateNode.h"
#include "Tag/LicenseProcessor.h"

// 前置声明
class ConfirmLicenseMission;

/**
 * @file ConfirmLicenseNodes.h
 * @brief 号牌确认状态机节点定义
 * 包含搜索号牌、确认号牌和重新定位号牌三种状态
 */

/**
 * @class SearchLicenseState
 * @brief 搜索号牌状态类
 * 负责在图像中搜索并定位号牌
 */
class SearchLicenseState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    SearchLicenseState(MissionNode& mission);
    ~SearchLicenseState()=default;

public:
    /**
     * @brief 进入状态时调用
     */
    void on_enter() override;
    
    /**
     * @brief 更新状态时调用，返回控制指令
     * @return 控制指令数组 [x, y, z, yaw]
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出状态时调用
     */
    void on_exit() override;

private:
    /**
     * @brief 号牌处理器，用于处理和识别号牌信息
     * 提供号牌检测、验证和比对功能
     */
    std::shared_ptr<LicenseProcessor> _license_processor;
    
    /**
     * @brief 移动向量，包含方向和距离
     * 第一个元素为方向字符('y', 'z')，第二个元素为移动方向
     */
    std::tuple<char, double> _move_vector;
};

/**
 * @class ConfirmLicenseState
 * @brief 确认号牌状态类
 * 负责确认检测到的号牌是否为目标号牌
 */
class ConfirmLicenseState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    ConfirmLicenseState(MissionNode& mission);
    ~ConfirmLicenseState()=default;

public:
    /**
     * @brief 进入状态时调用
     * 初始化确认过程所需的参数和状态
     */
    void on_enter() override;
    
    /**
     * @brief 更新状态时调用，返回控制指令
     * 验证检测到的号牌是否与目标号牌匹配
     * @return 控制指令数组 [x, y, z, yaw]
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出状态时调用
     * 清理确认过程中的临时变量和状态
     */
    void on_exit() override;

private:
    /**
     * @brief 号牌处理器，用于处理和识别号牌信息
     */
    std::shared_ptr<LicenseProcessor> _license_processor;
};
/**
 * @class RepositionLicenseState
 * @brief 重新定位号牌状态类
 * 当无法确认号牌时的应急处理状态，用于重新调整位置以再次尝试
 */
class RepositionLicenseState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    RepositionLicenseState(MissionNode& mission);
    ~RepositionLicenseState()=default;

public:
    /**
     * @brief 进入状态时调用
     */
    void on_enter() override;
    
    /**
     * @brief 更新状态时调用，返回控制指令
     * @return 控制指令数组 [x, y, z, yaw]
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出状态时调用
     */
    void on_exit() override;

private:
    /**
     * @brief 号牌处理器，用于处理和识别号牌信息
     * 提供号牌检测、验证和比对功能
     */
    std::shared_ptr<LicenseProcessor> _license_processor;
    
    /**
     * @brief 新的移动向量，用于重新定位
     * 第一个元素为方向字符('x', 'y', 'z')，第二个元素为移动距离和方向
     */
    std::tuple<char, double> _new_move_vector;
};