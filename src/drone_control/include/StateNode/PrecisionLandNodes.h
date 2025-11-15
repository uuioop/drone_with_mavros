#pragma once

#include <memory>
#include <unordered_map>

#include "Tag/ArucoTagProcessor.h"
#include "StateNode/StateNode.h"

// 前置声明
class PrecisionLandMission;

/**
 * @file PrecisionLandNodes.h
 * @brief 精确着陆状态机节点定义
 * 包含搜索标签、对准引导标签、接近引导标签、对准平台标签、下降和着陆等状态
 */

/**
 * @class SearchTagState
 * @brief 搜索标签状态类
 * 负责控制无人机搜索并发现ArUco引导标签
 */
class SearchTagState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    SearchTagState(MissionNode& mission);
    ~SearchTagState()=default;

    /**
     * @brief 进入状态时调用
     * 初始化搜索过程所需的参数和状态
     */
    void on_enter() override;
    
    /**
     * @brief 更新状态时调用，返回控制指令
     * 执行搜索操作，搜索并发现ArUco引导标签
     * @return 控制指令数组 [x, y, z, yaw]
     */
    std::array<double,4> on_update() override;
    
    /**
     * @brief 退出状态时调用
     * 清理搜索过程中的临时变量和状态
     */
    void on_exit() override;

private:
    /**
     * @brief 前置摄像头的标签处理器
     * 用于处理前置摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;
};

/**
 * @class AlignOnGuideTagState
 * @brief 对准引导标签状态类
 * 负责控制无人机对准到ArUco引导标签的中心位置
 */
class AlignOnGuideTagState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    AlignOnGuideTagState(MissionNode& mission);
    ~AlignOnGuideTagState()=default;

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
     * @brief 前置摄像头的标签处理器
     * 用于处理前置摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;
    
    /**
     * @brief 对准容差映射表，定义各方向的误差容忍范围
     * 存储x、y、z和偏航角各方向的允许误差值
     */
    std::unordered_map<std::string, double> _align_tolerance;
    
    /**
     * @brief 对准误差向量，存储各方向的误差值
     * 包含[x, y, z, yaw]四个方向的误差
     */
    std::array<double, 4> _alignment_errors;
};

/**
 * @class ApproachGuideTagState
 * @brief 接近引导标签状态类
 * 负责控制无人机向ArUco引导标签方向接近，同时开始使用下视摄像头搜索平台标签
 */
class ApproachGuideTagState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    ApproachGuideTagState(MissionNode& mission);
    ~ApproachGuideTagState()=default;

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
     * @brief 前置摄像头的标签处理器
     * 用于处理前置摄像头拍摄的图像中ArUco引导标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;
};

/**
 * @class AlignOnPlatformTagState
 * @brief 对准平台标签状态类
 * 负责控制无人机对准到平台上的ArUco标签的中心位置
 */
class AlignOnPlatformTagState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    AlignOnPlatformTagState(MissionNode& mission):StateNode(mission)
    {}
    ~AlignOnPlatformTagState()=default;

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
};

/**
 * @class DescendState
 * @brief 下降状态类
 * 负责控制无人机在对准平台标签后平稳下降
 */
class DescendState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    DescendState(MissionNode& mission);
    ~DescendState()=default;

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
     * @brief 平台标记对准误差向量，存储各方向的误差值
     * 包含[x, y, z, yaw]四个方向的误差
     */
    std::array<double, 4> _alignment_errors;
};

/**
 * @class LandState
 * @brief 着陆状态类
 * 负责控制无人机执行最终的着陆操作
 */
class LandState:public StateNode
{
public:
    /**
     * @brief 构造函数
     * @param mission 所在任务引用
     */
    LandState(MissionNode& mission):StateNode(mission)
    {}
    ~LandState()=default;

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
};
