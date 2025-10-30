/**
 * @file MavrosBridge.h
 * @brief MAVROS桥接器，提供与MAVROS通信的接口，用于控制无人机
 */
#pragma once
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <array>

#include "core/StatusMonitor.h"

/**
 * @class MavrosBridge
 * @brief MAVROS桥接器类，封装了与MAVROS通信的功能，提供无人机控制接口
 * @note mavros中本地位置指令是ENU坐标系，而不是NED坐标系
 * @note 机体坐标系下的速度指令是FLU坐标系，而不是FRD坐标系
 * @note mavros使用的是ros的坐标系而非PX4的坐标系
 */
class MavrosBridge
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄引用
     * @param status_monitor 状态监控器引用
     */
    MavrosBridge(ros::NodeHandle& nh, StatusMonitor& status_monitor);
    
    /**
     * @brief 析构函数
     */
    ~MavrosBridge();

    /**
     * @brief 连接到飞控单元(FCU)
     * @return 连接成功返回true
     */
    bool connect_to_fcu();
    
    /**
     * @brief 解锁无人机（准备飞行）
     * @return 解锁成功返回true
     */
    bool arm();
    
    /**
     * @brief 锁定无人机（停止飞行）
     * @return 锁定成功返回true
     */
    bool disarm();
    
    /**
     * @brief 切换无人机飞行模式
     * @param mode 飞行模式名称
     * @return 切换成功返回true
     */
    bool switch_flight_mode(const std::string& target_mode);
    
    /**
    * @brief 设置本地坐标系下的目标位置和姿态
    * @param pos std::array<double, 4> {x, y, z, yaw}
    * @param orient std::array<double, 4> {qx, qy, qz, qw}，如果使用，则忽略pos中的yaw
    * @note 坐标系为 ENU (East-North-Up)
    * @return 设置成功返回true
    */
    bool set_position_local(const std::array<double, 4>& pos,const std::array<double, 4>& orient={0.0,0.0,0.0,1.0});
    
    /**
     * @brief 设置全局坐标系下的目标位置
     * @param pos 全局位置消息
     * @return 设置成功返回true
     */
    bool set_position_global(const geographic_msgs::GeoPoseStamped& pos);
    
    /**
     * @brief 设置机体坐标系下的速度指令
     * @param vel 线速度和偏航角速率 {vx, vy, vz, yaw_rate}
     * @note 单位分别为m/s和度/s
     * @note 坐标系为FLU (Forward-Left-Up)
     * @return 设置成功返回true
     */
    bool set_velocity_body(const std::array<double, 4>& vel);

private:
    /**
     * @brief ROS节点句柄引用
     */
    ros::NodeHandle& _nh;
    
    /**
     * @brief 状态监控器引用
     */
    StatusMonitor& _status_monitor;
    
    /**
     * @brief 解锁/锁定服务客户端
     */
    ros::ServiceClient _arm_client;
    
    /**
     * @brief 设置模式服务客户端
     */
    ros::ServiceClient _set_mode_client;
    
    /**
     * @brief 本地位置发布者
     */
    ros::Publisher _local_pos_pub;
    
    /**
     * @brief 全局位置发布者
     */
    ros::Publisher _global_pos_pub;
    
    /**
     * @brief 机体速度发布者
     */
    ros::Publisher _body_vel_pub;

    /**
     * @brief 控制循环频率，默认20Hz
     */
    ros::Rate _rate{20.0};
    
private:
    /**
     * @brief 设置无人机解锁或上锁状态
     * @param flag true为解锁，false为锁定
     * @return 设置成功返回true
     */
    bool set_arm(bool flag);   

    /**
     * @brief 设置无人机飞行模式
     * @param mode 飞行模式名称
     * @return 设置成功返回true
     */
    bool set_mode(const std::string& mode);

    /**
     * @brief 启动离板控制模式(OFFBOARD)
     * @return 启动成功返回true
     */
    bool start_offboard_mode();

    /**
     * @brief 服务调用辅助模板函数
     * @tparam ServiceType 服务类型
     * @param client 服务客户端
     * @param service 服务对象
     * @param service_name 服务名称（用于错误日志）
     * @return 服务调用成功返回true
     */
    template<typename ServiceType>
    bool callService(ros::ServiceClient& client, 
                    ServiceType& service, 
                    const std::string& service_name)
    {
        if (client.call(service))
        {
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call %s service", service_name.c_str());
            return false;
        }
    }

    /**
     * @brief 消息发布辅助函数
     * @tparam MessageType 消息类型
     * @param publisher 消息发布者
     * @param message 消息对象
     * @param frame_id 坐标系ID，默认为"map"
     * @return 发布成功返回true
     */
    template<typename MessageType>
    bool publishMessage(ros::Publisher& publisher, 
                    MessageType& message, 
                    const std::string& frame_id = "map")
    {
        // 设置消息头
        message.header.stamp = ros::Time::now();
        message.header.frame_id = frame_id;
        
        // 发布消息
        publisher.publish(message);
        return true;
    }
};