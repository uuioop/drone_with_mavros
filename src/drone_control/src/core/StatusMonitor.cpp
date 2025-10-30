/**
 * @file StatusMonitor.cpp
 * @brief StatusMonitor类的实现文件，负责监控无人机状态
 */
#include "core/StatusMonitor.h"

/**
 * @brief StatusMonitor构造函数
 * @param nh ROS节点句柄引用
 * @note 初始化所有状态订阅器
 */
StatusMonitor::StatusMonitor(ros::NodeHandle& nh)
    : _nh(nh)
{
    _state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 10, &StatusMonitor::state_cb, this);
    _local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &StatusMonitor::local_pos_cb, this);
    _global_pos_sub = _nh.subscribe<geographic_msgs::GeoPoseStamped>("mavros/global_position/pose", 10, &StatusMonitor::global_pos_cb, this);
    _imu_data_sub = _nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &StatusMonitor::imu_data_cb, this);
    _landed_state_sub = _nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &StatusMonitor::landed_state_cb, this);
}
/**
 * @brief StatusMonitor析构函数
 * @note 使用默认实现
 */
StatusMonitor::~StatusMonitor()=default;

/**
 * @brief MAVROS状态回调函数
 * @param msg MAVROS状态消息指针
 * @note 更新连接状态、解锁状态和飞行模式
 * @note 使用互斥锁保护状态数据
 */
void StatusMonitor::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    // 检查是否连接
    if(!_connected && msg->connected)
    {
        _connected = true;
        ROS_INFO("Connected to FCU");
    }
    else if(_connected && !msg->connected)
    {
        _connected = false;
        ROS_INFO("Disconnected from FCU");
    }
    // 更新是否 armed
    if(!_armed && msg->armed)
    {
        _armed = true;
        ROS_INFO("Armed");
    }
    else if(_armed && !msg->armed)
    {
        _armed = false;
        ROS_INFO("Disarmed");
    }
    // 更新当前状态
    _current_state = *msg;
    if(msg->mode != _current_mode)
    {
        // 记录模式变化
        std::string last_mode = _current_mode;
        _current_mode = msg->mode;
        ROS_INFO("Mode %s changed to: %s", last_mode.c_str(), _current_mode.c_str());
    }

}

/**
 * @brief 本地位置回调函数
 * @param msg 本地位置消息指针
 * @note 更新无人机本地位置信息
 * @note 使用互斥锁保护状态数据
 */
void StatusMonitor::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _local_pos = *msg;
}

/**
 * @brief 全局位置回调函数
 * @param msg 全局位置消息指针
 * @note 更新无人机全局位置信息
 * @note 使用互斥锁保护状态数据
 */
void StatusMonitor::global_pos_cb(const geographic_msgs::GeoPoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _global_pos = *msg;
}

/**
 * @brief IMU数据回调函数
 * @param msg IMU数据消息指针
 * @note 更新无人机姿态信息
 * @note 使用互斥锁保护状态数据
 */
void StatusMonitor::imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _orientation = msg->orientation;
}

/**
 * @brief 着陆状态回调函数
 * @param msg 扩展状态消息指针
 * @note 更新无人机着陆状态
 * @note 使用互斥锁保护状态数据
 */
void StatusMonitor::landed_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _landed_state = msg->landed_state;
}
