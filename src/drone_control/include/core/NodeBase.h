#pragma once
#include <array>
#include <ros/ros.h>

/**
 * @file NodeBase.h
 * @brief 节点基类定义
 * 
 * 该文件定义了节点基类，作为StateNode和MissionNode的共同基类，
 * 提供了进入、退出和更新等通用接口。
 */

/**
 * @brief 节点基类
 * 
 * 所有状态机和任务机中的节点都应继承自此类，提供统一的接口。
 */
class NodeBase
{
public:
    /**
     * @brief 构造函数
     */
    NodeBase() = default;
    
    /**
     * @brief 析构函数
     */
    virtual ~NodeBase() = default;

    /**
     * @brief 进入节点时执行的函数
     * 
     * 虚函数，派生类可以重写，用于初始化节点相关的变量和行为
     */
    virtual void on_enter() = 0;
    
    /**
     * @brief 退出节点时执行的函数
     * 
     * 虚纯函数，派生类必须实现，用于清理节点相关的资源和行为
     */
    virtual void on_exit() = 0;
    
    /**
     * @brief 更新节点时执行的函数
     * 
     * 虚函数，派生类可以重写，用于执行节点的主要逻辑
     * 
     * @return std::array<double,4> 包含x,y,z,yaw速度指令的数组
     * @note 为了兼顾使用板外模式的状态（返回速度指令），任务及状态的更新函数都返回数组类型
     */
    virtual std::array<double,4> on_update() = 0;
};