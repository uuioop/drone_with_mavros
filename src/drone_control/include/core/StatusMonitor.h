/**
 * @file StatusMonitor.h
 * @brief 无人机状态监控器，用于订阅和管理无人机的各种状态信息
 */
#pragma once
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/ExtendedState.h>
#include <string>
#include <mutex>

/**
 * @class StatusMonitor
 * @brief 无人机状态监控器类，负责订阅、存储和提供无人机的各种状态信息
 */
class StatusMonitor
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄引用
     */
    StatusMonitor(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~StatusMonitor();

    /**
     * @brief 检查无人机是否已连接
     * @return 如果连接返回true，否则返回false
     */
    bool is_connected() const
    {
        // 加锁保护状态变量的访问
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _connected;
    }
    
    /**
     * @brief 检查无人机是否已解锁（可飞行状态）
     * @return 如果已解锁返回true，否则返回false
     */
    bool is_armed() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);    
        return _armed;
    }

    /**
     * @brief 检查无人机是否准备好飞行（已连接且已解锁）
     * @return 如果准备好飞行返回true，否则返回false
     */
    bool is_ready_for_fly() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _connected && _armed;
    }

    /**
     * @brief 获取无人机的着陆状态
     * @return 着陆状态代码
     */
    uint8_t get_landed_state() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _landed_state;
    }

    /**
     * @brief 获取无人机当前的飞行模式
     * @return 飞行模式名称
     */
    std::string get_flight_mode() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _current_mode;
    }

    /**
     * @brief 获取无人机的姿态四元数
     * @return 姿态四元数
     */
    geometry_msgs::Quaternion get_orientation() const
    {   
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _orientation;
    }
    
    /**
     * @brief 获取无人机当前的完整状态
     * @return 无人机状态消息
     */
    mavros_msgs::State get_current_state() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _current_state;
    }

    /**
     * @brief 获取无人机的本地位置
     * @return 本地位置消息
     */
    geometry_msgs::PoseStamped get_local_pos() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _local_pos;
    }
    
    /**
     * @brief 获取无人机的全局位置
     * @return 全局位置消息
     */
    geographic_msgs::GeoPoseStamped get_global_pos() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _global_pos;
    }

private:
    /**
     * @brief ROS节点句柄引用
     */
    ros::NodeHandle& _nh;
    
    /**
     * @brief 无人机状态订阅器
     */
    ros::Subscriber _state_sub;
    
    /**
     * @brief 本地位置订阅器
     */
    ros::Subscriber _local_pos_sub;
    
    /**
     * @brief 全局位置订阅器
     */
    ros::Subscriber _global_pos_sub;
    
    /**
     * @brief IMU数据订阅器
     */
    ros::Subscriber _imu_data_sub;
    
    /**
     * @brief 着陆状态订阅器
     */
    ros::Subscriber _landed_state_sub;
    
    /**
     * @brief 无人机当前状态
     */
    mavros_msgs::State _current_state;
    
    /**
     * @brief 本地位置（ENU坐标系）
     */
    geometry_msgs::PoseStamped _local_pos;
    
    /**
     * @brief 全局位置（经纬度高度）
     */
    geographic_msgs::GeoPoseStamped _global_pos;
    
    /**
     * @brief 无人机姿态四元数
     */
    geometry_msgs::Quaternion _orientation;
    
    /**
     * @brief 无人机当前飞行模式
     */
    std::string _current_mode = "UNKOWN";
    
    /**
     * @brief 连接状态标志
     */
    bool _connected = false;
    
    /**
     * @brief 解锁状态标志
     */
    bool _armed = false;
    
    /**
     * @brief 着陆状态代码
     */
    uint8_t _landed_state = 0;
    
    /**
     * @brief 状态互斥锁，用于保护状态变量的并发访问
     * @note mutable关键字允许在const成员函数中锁定和解锁互斥锁
     */
    mutable std::mutex _status_mutex;

private:
    /**
     * @brief 状态消息回调函数
     * @param msg 无人机状态消息
     */
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    
    /**
     * @brief 本地位置消息回调函数
     * @param msg 本地位置消息
     */
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    /**
     * @brief 全局位置消息回调函数
     * @param msg 全局位置消息
     */
    void global_pos_cb(const geographic_msgs::GeoPoseStamped::ConstPtr& msg);
    
    /**
     * @brief IMU数据回调函数
     * @param msg IMU数据消息
     */
    void imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg);
    
    /**
     * @brief 着陆状态消息回调函数
     * @param msg 扩展状态消息，包含着陆状态
     */
    void landed_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);

};