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

protected:
    /** @brief 此节点的超时时长（秒），0.0表示不启用超时 */
    ros::Duration _timeout_duration=ros::Duration(0.0);
    /** @brief 进入此节点时的时间戳 */
    ros::Time _entry_time;
protected:
    /**
     * @brief 检查节点是否超时
     * 
     * 判断从进入节点到现在的时间是否超过了配置的超时时间
     * 
     * @return true 如果节点已超时
     * @return false 如果节点未超时
     */
    bool is_timeout() const
    {
        if(_timeout_duration.toSec()<=0.0) return false;
        // 如果进入时间或当前时间为0，说明时间尚未初始化，不应触发超时
        if (_entry_time.toSec() == 0.0 || ros::Time::now().toSec() == 0.0) 
        {
            return false;
        }
        return (ros::Time::now()-_entry_time)>_timeout_duration;
    }
};