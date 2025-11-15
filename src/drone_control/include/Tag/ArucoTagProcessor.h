#pragma once
#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <limits>
#include <string>
#include <array>
#include <vector>
#include <unordered_map>

#include "Tag/ArucoTag.h"

/**
 * @file ArucoTagProcessor.h
 * @brief ArUco标记处理器类定义
 * 
 * 该文件定义了ArucoTagProcessor类，负责处理ArUco标记的检测数据，
 * 计算标记的有效性、对准状态以及生成速度控制指令。
 */

/**
 * @brief ArUco标记处理器类
 * 
 * 负责订阅和处理ArUco标记的位姿信息，计算标记的对准状态，
 * 并生成相应的速度控制指令，用于无人机的精确着陆或其他视觉导航任务。
 */
class ArucoTagProcessor
{
public:
    /**
     * @brief 构造函数
     * 
     * @param nh ROS节点句柄
     * @param tag_config 标记配置参数
     */
    ArucoTagProcessor(ros::NodeHandle& nh, const TagConfig tag_config);
    
    /**
     * @brief 析构函数
     */
    ~ArucoTagProcessor();
    
public:
    /** @brief 是否为父标记或复合标记 */
    TagType get_type() const { return _tag_config.type; }

    /**
     * @brief 设置处理器启动状态
     * 
     * @param flag 启动状态
     */
    void set_start(bool flag) { _is_started = flag; }
    
    /**
     * @brief 更新并记录标记有效性
     * 
     * 检查标记是否超时，更新当前有效性状态，并记录变化
     */
    void update_and_log_validity();
    
    /**
     * @brief 更新标记配置
     * 
     * @param config 新的标记配置
     */
    void update_tagconfig(const TagConfig config) { _tag_config = config; }
    
    /**
     * @brief 检查标记是否有效
     * 
     * @return true 如果标记当前有效
     * @return false 如果标记当前无效
     */
    bool is_valid() const { return _is_valid_now; }
    
    /**
     * @brief 检查标记是否已对准
     * 
     * 判断无人机是否已经对准到标记的允许误差范围内
     * 
     * @return true 如果已对准
     * @return false 如果未对准
     */
    bool is_aligned() const;
    
    /**
     * @brief 获取对齐误差向量
     * 
     * 返回在x, y, z轴和偏航角上的对准误差
     * 
     * @return std::array<double,4> 包含x,y,z,yaw误差的数组
     */
    std::array<double,4> get_alignment_errors() const;
    
    /**
     * @brief 计算速度指令
     * 
     * 基于当前复合标记位置和对准状态，计算无人机的速度控制指令
     * 
     * @param is_yaw 是否包含偏航控制，默认为true
     * @return std::array<double,4> 包含x,y,z,yaw速度指令的数组
     */
    std::array<double,4> calculate_velocity_command(bool is_yaw= true) const;

private:
    /** @brief ROS节点句柄 */
    ros::NodeHandle _nh;
    /** @brief 自定义包标记位姿订阅者，接收标记检测结果 */
    ros::Subscriber _pose_sub;
    /** @brief 社区包标记位姿订阅者，接收标记检测结果 */
    ros::Subscriber _fiducial_sub;
    /** @brief 标记配置参数 */
    TagConfig _tag_config;
    /** @brief 当前检测到的标记 */
    ArucoTag _tag;
    /** @brief 当前标记是否有效 */
    bool _is_valid_now = false;
    /** @brief 上一帧标记是否有效，用于检测状态变化 */
    bool _was_previously_valid = false;
    /** @brief 对准定时器，用于跟踪对准持续次数（mutable允许在const函数中修改） */
    mutable int _align_timer = 0;
    /** @brief 过滤后的偏航角误差（度） */
    mutable double _filtered_yaw_error_deg = 0.0;
    /** @brief 过滤系数，用于平滑偏航角速度 */
    double _filter_alpha = 0.5;
    /** @brief 标记处理器是否已启动 */
    bool _is_started = false;

private:
    /**
     * @brief 检查标记与目标的距离是否有效
     * 
     * @param tag 要检查的标记
     * @return true 如果距离有效
     * @return false 如果距离无效
     */
    bool check_distance_valid(const ArucoTag& tag) const;

    /**
     * @brief 目标位姿回调函数
     * 
     * 处理从aruco_detect接收到的标记位姿信息，将其转换为机身坐标系
     * 
     * @param msg 包含标记位姿的ROS消息（fiducial_transform类型）
     */
    void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    /**
     * @brief 目标位姿回调函数
     * 
     * 处理从board_detect接收到的板位姿信息，将其转换为机身坐标系
     * 
     * @param msg 包含板位姿的ROS消息（PoseStamped类型）
     */
    void board_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    /**
     * @brief 将相机坐标系中的标记转换为机体坐标系
     * 
     * @param tag_cam 相机坐标系中的标记
     * @return ArucoTag 转换到机体坐标系后的标记
     */
    ArucoTag get_tag_body(const ArucoTag& tag_cam) const;
    
    /**
     * @brief 检查标记是否超时
     * 
     * 判断从上次有效检测到现在的时间是否超过配置的超时时间
     * 
     * @return true 如果标记已超时
     * @return false 如果标记未超时
     */
    bool check_tag_timeout() const;
    
    /**
     * @brief 获取偏航角误差（度）
     * 
     * 计算当前偏航角与目标偏航角的误差
     * 
     * @return double 偏航角误差（度）
     */
    double get_yaw_error_deg() const;
    
    /**
     * @brief 计算偏航角速度指令
     * 
     * 根据当前偏航角误差，计算相应的偏航角速度控制指令
     * 
     * @return double 偏航角速度指令（度/秒）
     */
    double calculate_yaw_rate() const;
};