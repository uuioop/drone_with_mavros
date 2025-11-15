#pragma once

#include <memory>
#include <unordered_map>

#include "Tag/ArucoTagProcessor.h"
#include "StateNode/StateNode.h"

class SafeDepartureMission;

class FindFrontTag:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    FindFrontTag(MissionNode& mission);
    ~FindFrontTag()=default;

    /**
     * @brief 进入节点时调用
     * 初始化起飞过程所需的参数和状态
     */
    void on_enter() override;
    
    /**
     * @brief 更新节点时调用，返回控制指令
     * 执行起飞操作，使无人机起飞
     * @return 控制指令数组 [x, y, z, yaw]
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出节点时调用
     * 清理起飞过程中的临时变量和状态
     */
    void on_exit() override;

private:
    /**
     * @brief 前置摄像头的标签处理器
     * 用于处理前置摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;

    /**
     * @brief 下方摄像头的标签处理器
     * 用于处理下方摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _down_tag_processor;

    /**
     * @brief 引导标记的对准误差向量
     * 用于存储引导标记与无人机的对准误差向量
     */
    std::array<double,4> _alignment_errors;
};

class KeepAwayFromTag:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    KeepAwayFromTag(MissionNode& mission);
    ~KeepAwayFromTag()=default;
    /**
     * @brief 进入节点时调用
     * 初始化离开过程所需的参数和状态
     */
    void on_enter() override;
    
    /**
     * @brief 更新节点时调用，返回控制指令
     * 执行离开操作，使无人机离开
     * @return 控制指令数组 [x, y, z, yaw]
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出节点时调用
     * 清理离开过程中的临时变量和状态
     */
    void on_exit() override;
private:
    /**
     * @brief 前置摄像头的标签处理器
     * 用于处理前置摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;

    /**
     * @brief 引导标记的对准误差向量
     * 用于存储引导标记与无人机的对准误差向量
     */
    std::array<double,4> _alignment_errors;
};

class BlindState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    BlindState(MissionNode& mission);
    ~BlindState()=default;
    /**
     * @brief 进入节点时调用
     * 初始化盲飞过程所需的参数和状态
     */
    void on_enter() override;
    
    /**
     * @brief 更新节点时调用，返回控制指令
     * 执行盲飞操作，使无人机盲飞
     * @return 控制指令数组 [x, y, z, yaw]
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出节点时调用
     * 清理盲飞过程中的临时变量和状态
     */
    void on_exit() override;
private:
    /**
     * @brief 前置摄像头的标签处理器
     * 用于处理前置摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;
};

