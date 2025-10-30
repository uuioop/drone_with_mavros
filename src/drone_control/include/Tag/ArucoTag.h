#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <limits>
#include <ros/ros.h>

/**
 * @file ArucoTag.h
 * @brief ArUco标记相关的数据结构定义
 * 
 * 该文件定义了用于表示ArUco标记的位置、姿态信息的结构体，
 * 以及标记检测和控制所需的配置参数结构体。
 */

/**
 * @brief ArUco标记结构体
 * 
 * 用于存储检测到的ArUco标记的位置、姿态和时间戳信息。
 */
struct ArucoTag
{
    /** @brief 标记在机体坐标系中的位置，初始值为NaN */
    Eigen::Vector3d position=Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    /** @brief 标记在机体坐标系中的姿态四元数 */
    Eigen::Quaterniond orientation;
    /** @brief 标记检测的时间戳 */
    ros::Time timestamp =ros::Time(0);

    /**
     * @brief 检查标记数据是否有效
     * 
     * 通过判断时间戳是否大于0来确定数据是否已被更新
     * 
     * @return true 如果标记数据有效
     * @return false 如果标记数据无效
     */
    bool is_valid() const
    { 
        return timestamp.toSec() > 0.0; 
    }
};

/**
 * @brief 标记配置结构体
 * 
 * 用于存储ArUco标记检测和控制的配置参数，所有参数基于机体坐标系
 */
struct TagConfig {
    /** @brief 订阅的主题名称，用于接收标记检测结果 */
    std::string topic_name="/aruco_detect/pose";
    /** @brief 目标ID，用于指定需要控制的特定标记 */
    int target_id=0;
    /** @brief 标记名称标识，用于区分不同标记 */
    std::string tag_name="default";
    /** @brief 相机到机体的旋转矩阵，用于坐标变换 */
    Eigen::Matrix3d rotation_matrix =Eigen::Matrix3d::Identity();
    /** @brief 对准轴，指定需要控制的轴向 */
    std::vector<char> alignment_axes={'x','y','\0'};
    /** @brief 最大检测距离，超过此距离的标记将被忽略 */
    std::array<double,3> max_detection_distance={
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    };
    /** @brief 对准容差，确定标记对准的精度要求 */
    double aligned_tolerance=0.2;
    /** @brief 偏航角容差，以度为单位 */
    double yaw_tolerance_deg=20.0;
    /** @brief 最大偏航角速度，以度/秒为单位 */
    double max_yaw_rate_deg_s=10.0;
    /** @brief 最小偏航角速度，以度/秒为单位 */
    double min_yaw_rate_deg_s=2.0;
    /** @brief 目标超时时间，超过此时间未检测到标记将认为丢失 */
    double target_timeout=3.0;
    /** @brief 目标偏移量，用于调整目标位置 */
    Eigen::Vector3d target_offset=Eigen::Vector3d::Zero();
    /** @brief PID控制器的比例系数 */
    double kp=0.5;
    /** @brief PID控制器的积分系数 */
    double ki=0.0;
    /** @brief PID控制器的微分系数 */
    double kd=0.0;
    /** @brief 最大移动速度，限制控制输出 */
    double max_speed=0.6;
};