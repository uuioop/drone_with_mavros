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
    // 订阅 MAVROS 状态话题
    _state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 10, &StatusMonitor::state_cb, this);
    // 订阅 本地位置话题
    _local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &StatusMonitor::local_pos_cb, this);
    // 订阅 NavSatFix 话题
    _gps_sub = _nh.subscribe("/mavros/global_position/global", 10, &StatusMonitor::gps_cb, this);
    // 订阅 rel_alt 话题
    _rel_alt_sub = _nh.subscribe("/mavros/global_position/rel_alt", 10, &StatusMonitor::rel_alt_cb, this);
    // 订阅 mission_list 话题
    _mission_list_sub = _nh.subscribe("/mavros/mission/waypoints", 10, &StatusMonitor::mission_list_cb, this);
    // 订阅 mission_reached 话题
    _mission_reached_sub = _nh.subscribe("/mavros/mission/reached", 10, &StatusMonitor::mission_reached_cb, this);
    // 订阅 IMU 话题
    _imu_data_sub = _nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &StatusMonitor::imu_data_cb, this);
    // 订阅 landed_state 话题
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
        // 条件：
        // 1. 新模式不是 AUTO.MISSION
        // 2. 曾经加载过一个任务 (_total_waypoints > 0)
        // 3. 任务尚未完成 (_mission_completed == false)
        // 4. 之前没有报告过中止 (_mission_aborted == false)
        if (_current_mode != "AUTO.MISSION" && _total_waypoints > 0 && !_mission_completed && !_mission_aborted) {
            _mission_aborted = true; // 设置中止标志，防止重复报告
            ROS_WARN("Mission aborted! Switched to mode: %s", _current_mode.c_str());
        }
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
 * @brief GPS位置回调函数
 * @param msg 全局位置消息指针
 * @note 更新无人机全局位置信息（经度、纬度）
 * @note 使用互斥锁保护状态数据
 */
void StatusMonitor::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _global_pos[0] = msg->latitude;
    _global_pos[1] = msg->longitude;
    _global_pos[2] = msg->altitude;
    // 只有在第一次接收到有效的GPS数据时打印一次
    if (!_gps_received_fix && msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
        _gps_received_fix = true;
        ROS_INFO("GPS fix received!");
    }
}

/**
 * @brief 相对高度回调函数
 * @param msg 相对高度消息指针
 * @note 更新无人机相对高度信息
 * @note 使用互斥锁保护状态数据
 */
void StatusMonitor::rel_alt_cb(const std_msgs::Float64::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _rel_alt = msg->data;
}

/**
 * @brief 任务列表回调函数
 * @param msg 任务列表消息指针
 * @note 更新任务的总航点数
 */
void StatusMonitor::mission_list_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    size_t new_total_waypoints = msg->waypoints.size();

    if (_total_waypoints != new_total_waypoints) {
        ROS_INFO("Mission list updated. Total waypoints changed from %zu to %zu.", _total_waypoints, new_total_waypoints);
        _total_waypoints = new_total_waypoints;
        
        // 如果任务被清空，我们也应该重置状态
        if (_total_waypoints == 0) {
            _mission_completed = false;
            _mission_aborted = false;
            _reached_waypoint_index = -1;
        }
    }
}

/**
 * @brief 航点到达回调函数
 * @param msg 航点到达消息指针
 * @note 监控任务进度，判断任务是否完成
 */
void StatusMonitor::mission_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    // 1. 如果任务已经完成或中止，或者没有任务在进行，则忽略此消息
    if (_mission_completed || _mission_aborted || _total_waypoints == 0) {
        return;
    }

    // 2. 如果收到的航点索引不合理（例如小于当前已到达的），也忽略
    if (msg->wp_seq < _reached_waypoint_index) {
        ROS_WARN("Received an out-of-order mission reached message. Ignored.");
        return;
    }
    
    if (msg->wp_seq != _reached_waypoint_index) 
    {
        _reached_waypoint_index = msg->wp_seq;
        ROS_INFO(">>> Progress: Reached Waypoint %d of %zu <<<", _reached_waypoint_index + 1, _total_waypoints);
    }
    // 判断任务是否完成 (航点索引从0开始)
    if (_reached_waypoint_index >= 0 && _reached_waypoint_index == _total_waypoints - 1) {
        if (!_mission_completed) 
        {
            _mission_completed = true;
            ROS_INFO("================ MISSION COMPLETE ================");
            // 在这里，你可以触发一个事件，或者设置一个标志
            // 让其他模块知道任务已经完成。
        }
    }
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
