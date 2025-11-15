#pragma once
#include <memory>

#include "MissionNode/MissionNode.h"
#include "core/NodeMachine.h"
#include "Tag/ArucoTagProcessor.h"
#include "Tag/ArucoTag.h"

/**
 * @file PrecisionLandMission.h
 * @brief 精确着陆任务头文件
 * 负责管理无人机使用ArUco标签进行精确着陆的过程
 */
/**
 * @class PrecisionLandMission
 * @brief 精确着陆任务类
 * 继承自MissionNode基类，实现无人机基于ArUco标签的精确着陆控制逻辑
 */
class PrecisionLandMission : public MissionNode
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param status_monitor 状态监控器引用
     * @param mavros_bridge MAVROS桥接器引用
     */
    PrecisionLandMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge);
    
    /**
     * @brief 析构函数
     */
    ~PrecisionLandMission();

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
     * @brief 获取前置摄像头的标签处理器
     * @return 前置标签处理器智能指针
     */
    std::shared_ptr<ArucoTagProcessor> get_front_tag_processor() const
    {
        return _front_tag_processor;
    }
    
    /**
     * @brief 获取下视摄像头的标签处理器
     * @param use_parent 是否优先使用父节点处理器
     * @return 下视标签处理器智能指针
     */
    std::shared_ptr<ArucoTagProcessor> get_down_tag_processor(bool use_parent =false) const
    {
        // 返回有效处理器，优先选择父节点
        return (_down_tag_parent_processor->is_valid() || use_parent)  ? _down_tag_parent_processor : _down_tag_child_processor;
    }
    
private:
    /**
     * @brief 状态机，管理精确着陆过程中的各个状态
     * 负责协调不同着陆阶段之间的转换和执行
     */
    NodeMachine _state_machine;
    
    /**
     * @brief 前置摄像头的标签配置信息
     * 存储前置摄像头检测ArUco标签的相关参数
     */
    TagConfig _front_tag_config;
    
    /**
     * @brief 下视摄像头父节点的标签配置信息
     * 存储下视摄像头父节点检测ArUco标签的相关参数
     */
    TagConfig _down_tag_parent_config;

    /**
     * @brief 下视摄像头子节点的标签配置信息
     * 存储下视摄像头子节点检测ArUco标签的相关参数
     */
    TagConfig _down_tag_child_config;
    
    /**
     * @brief 前置摄像头的标签处理器
     * 处理前置摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;
    
    /**
     * @brief 下视摄像头父节点的标签处理器
     * 处理下视摄像头父节点拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _down_tag_parent_processor;
    
    /**
     * @brief 下视摄像头子节点的标签处理器
     * 处理下视摄像头子节点拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _down_tag_child_processor;

private:
    /**
     * @brief 注册状态机中的各个状态
     * 初始化并注册精确着陆过程中所需的各种状态到状态机中
     */
    void register_states();
};
