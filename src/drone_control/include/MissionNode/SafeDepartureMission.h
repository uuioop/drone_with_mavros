#pragma once
#include <memory>

#include "MissionNode/MissionNode.h"
#include "core/NodeMachine.h"
#include "Tag/ArucoTagProcessor.h"
#include "Tag/ArucoTag.h"


class SafeDepartureMission : public MissionNode
{
public:
    /**
     * @brief 构造函数
     * @param drone_control 无人机控制引用
     * @param nh ROS节点句柄引用
     * @param status_monitor 状态监控器引用
     * @param mavros_bridge MAVROS桥接器引用
     */
    SafeDepartureMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge);
    /**
     * @brief 析构函数
     */
    ~SafeDepartureMission();

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
     * @return 下视标签处理器智能指针
     */
    std::shared_ptr<ArucoTagProcessor> get_down_tag_processor() const
    {
        return _down_tag_processor;
    }

private:
    NodeMachine _state_machine;
        
    /**
     * @brief 前置摄像头的标签配置信息
     * 存储前置摄像头检测ArUco标签的相关参数
     */
    TagConfig _front_tag_config;

    /**
     * @brief 下视摄像头的标签配置信息
     * 存储下视摄像头检测ArUco标签的相关参数
     */
    TagConfig _down_tag_config;

    /**
     * @brief 前置摄像头的标签处理器
     * 处理前置摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _front_tag_processor;

    /**
     * @brief 下视摄像头的标签处理器
     * 处理下视摄像头拍摄的图像中ArUco标签的检测和定位
     */
    std::shared_ptr<ArucoTagProcessor> _down_tag_processor;
    
private:
    /**
     * @brief 注册状态机中的各个状态
     * 初始化并注册精确着陆过程中所需的各种状态到状态机中
     */
    void register_states();
};
