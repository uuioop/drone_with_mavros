/**
 * @file NodeMachine.h
 * @brief 节点机基类定义
 * 
 * 该文件定义了节点机基类,可用于管理以NodeBase为基类或派生自NodeBase的类(如StateNode或MissionNode)的节点
 * 提供了节点管理、节点切换等通用功能。
 */
#pragma once
#include <string>
#include <unordered_map>
#include <memory>

#include "core/NodeBase.h"
/**
 * @brief 节点机基类
 * 
 * 节点机类，用于管理节点的注册、切换和更新，提供节点转换的基础设施。
 * 
 */
class NodeMachine
{
public:
    /**
     * @brief 构造函数
     */
    NodeMachine()=default;
    
    /**
     * @brief 析构函数
     */
    virtual ~NodeMachine() = default;
    
    /**
     * @brief 设置节点机的入口节点
     * 
     * @param id 入口节点的唯一标识符
     */
    void set_entry(const std::string& id)
    {
        _current_node=_node_pool[id];
        _need_init=true;
    }
    
    /**
     * @brief 切换到指定的节点
     * 
     * @param id 目标节点的唯一标识符
     */
    void switch_node(const std::string& id)
    {
        if (_current_node)
            _current_node->on_exit();
        _current_node=_node_pool[id];
        if (_current_node)
            _current_node->on_enter();
    }
    
    /**
     * @brief 注册节点
     * 
     * 将节点添加到节点机的节点池中
     * 
     * @param id 节点的唯一标识符
     * @param node 节点的智能指针
     */
    void register_node(const std::string& id, std::shared_ptr<NodeBase> node)
    {
        _node_pool[id]=node;
    }

    /**
     * @brief 更新当前活动节点
     * 
     * 调用当前活动节点的update()方法，更新其状态。
     * 
     * @return std::array<double,4> 包含x,y,z,yaw速度指令的数组
     * @note 为了兼顾使用板外模式的状态（返回速度指令），任务机及状态机的更新函数都返回数组类型
     */
    std::array<double,4> on_update()
    {
        if (!_current_node)
            return {};
        if (_need_init)
        {
            _need_init = false;
            if (_current_node) _current_node->on_enter();
        }
        return _current_node->on_update();
    }

    /** @brief 重置节点机状态，准备重新初始化 */
    void reset()
    {
        _need_init = true;
    }

private:
    /** @brief 标识节点机是否需要初始化 */
    bool _need_init;
    /** @brief 节点池，存储所有已注册的节点 */
    std::unordered_map<std::string, std::shared_ptr<NodeBase>> _node_pool;
    /** @brief 当前活动节点的智能指针 */
    std::shared_ptr<NodeBase> _current_node;
};