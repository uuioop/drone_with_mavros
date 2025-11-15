#pragma once
#include<array>

#include "core/NodeBase.h"

/**
 * @file StateNode.h
 * @brief 状态节点基类定义
 * 
 * 该文件定义了状态节点的抽象基类，作为状态机中所有具体状态的基类，
 * 提供了状态转换和行为执行的接口。
 */

// 声明任务节点类
class MissionNode;
/**
 * @brief 状态节点基类
 * 
 * 所有状态机中的具体状态都应继承自此类，实现各自的进入、更新和退出行为。
 * 提供了超时检测等通用功能，以及与所在任务的交互机制。
 */
class StateNode : public NodeBase
{
public:
    /**
     * @brief 构造函数
     * 
     * @param mission 任务引用，用于状态访问任务功能
     */
    StateNode(MissionNode& mission):_mission(mission)
    {}
    
    /**
     * @brief 析构函数
     * 
     * 使用虚析构函数确保派生类对象正确析构
     */
    virtual ~StateNode()=default;

    /**
     * @brief 进入状态时执行的函数
     * 
     * 虚纯函数，派生类必须实现，用于初始化状态相关的变量和行为
     */
    virtual void on_enter()=0;
    
    /**
     * @brief 退出状态时执行的函数
     * 
     * 虚纯函数，派生类必须实现，用于清理状态相关的资源和行为
     */
    virtual void on_exit()=0;
    
    /**
     * @brief 更新状态时执行的函数
     * 
     * 虚纯函数，派生类必须实现，用于执行状态的主要逻辑并返回控制指令
     * 
     * @return std::array<double,4> 包含x,y,z,yaw速度指令的数组
     */
    virtual std::array<double,4> on_update() override = 0;

protected:
    /** @brief 任务引用，用于状态访问任务功能 */
    MissionNode& _mission;
    /** @brief 速度指令数组 [x,y,z,yaw] */
    std::array<double,4> _vel_cmd={0.0,0.0,0.0,0.0};

};
