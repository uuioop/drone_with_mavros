/**
 * @file StatusMonitor.h
 * @brief 无人机状态监控器，用于订阅和管理无人机的各种状态信息
 */
#pragma once
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/WaypointList.h> // 用于接收当前航点列表
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/NavSatFix.h> // 用于GPS经纬度
#include <std_msgs/Float64.h>      // 用于相对高度
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/ExtendedState.h>
#include <string>
#include <mutex>
#include <limits>
#include <array>

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
     * @brief 获取无人机的全局位置（经纬度+绝对海拔高度）
     * @return 全局位置消息（{经度, 纬度, 绝对海拔高度}）
     */
    std::array<double, 3> get_global_pos() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _global_pos;
    }
    
    /**
     * @brief 检查GPS是否已接收固定位置
     * @return 如果已接收固定位置返回true，否则返回false
     */
    bool is_gps_fixed() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _gps_received_fix;
    }

    /**
     * @brief 检查任务是否已完成
     * @return 如果任务已完成返回true，否则返回false
     */
    bool is_mission_completed() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _mission_completed;
    }

    /**
     * @brief 检查任务是否已中止
     * @return 如果任务已中止返回true，否则返回false
     */
    bool is_mission_aborted() const
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        return _mission_aborted;
    }

    /**
     * @brief 重置GPS固定位置标志
     */
    void reset_gps_fix()
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        _gps_received_fix = false;
    }

    /**
     * @brief 重置任务相关标志（任务完成、任务中止、已到达航点索引）
     */
    void reset_mission_flags()
    {
        ROS_INFO("Mission flags have been reset manually.");
        std::lock_guard<std::mutex> lock(_status_mutex);
        _reached_waypoint_index = -1;
        _mission_completed = false;
        _mission_aborted = false;
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
     * @brief GPS位置（绝对海拔高度）订阅器
     */
    ros::Subscriber _gps_sub;

    /**
     * @brief 相对高度(Home点)订阅器
     */
    ros::Subscriber _rel_alt_sub;

    /**
     * @brief 任务列表订阅器
     */
    ros::Subscriber _mission_list_sub;

    /**
     * @brief 任务完成订阅器
     */
    ros::Subscriber _mission_reached_sub;
    
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
     * @brief 全局位置（经纬度+绝对海拔高度）
     */
    std::array<double, 3> _global_pos;
    
    /**
     * @brief GPS是否已接收固定位置
     */
    bool _gps_received_fix = false;
    
    /**
     * @brief 相对高度(Home点)
     */
    double _rel_alt = std::numeric_limits<double>::quiet_NaN();

    /**
     * @brief 任务总航点数
     */
    size_t _total_waypoints = 0;

    /**
     * @brief 当前已到达的航点索引（从0开始）
     */
    int _reached_waypoint_index = -1;
    
    /**
     * @brief 任务是否被中止的标志
     */
    bool _mission_aborted = false;

    /**
     * @brief 任务是否已完成的标志
     */
    bool _mission_completed = false;
    
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
     * @brief GPS位置消息回调函数
     * @param msg GPS位置消息
     */
    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    
    /**
     * @brief 相对高度(Home点)回调函数
     * @param msg 相对高度消息
     */
    void rel_alt_cb(const std_msgs::Float64::ConstPtr& msg);
    
    /**
     * @brief 任务列表回调函数
     * @param msg 任务列表消息
     */
    void mission_list_cb(const mavros_msgs::WaypointList::ConstPtr& msg);
    
    /**
     * @brief 任务完成回调函数
     * @param msg 任务完成消息
     */
    void mission_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg);
    
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