#pragma once

#include <ros/ros.h>
#include <my_interfaces/LicenseInfo.h>
#include <tuple>
#include <string>
#include <vector>
#include <set>
#include <map>

#include "Tag/License.h"

/**
 * @file LicenseProcessor.h
 * @brief 号牌处理器类定义
 * 
 * 该文件定义了LicenseProcessor类，负责处理号牌检测数据，
 * 验证号牌有效性，比较是否为目标号牌，并生成相应的控制指令。
 */

/**
 * @brief 号牌处理器类
 * 
 * 负责订阅和处理号牌检测信息，验证号牌有效性，
 * 比较当前号牌与目标号牌，并计算用于控制无人机对准号牌的指令。
 */
class LicenseProcessor{
public:
    /**
     * @brief 构造函数
     * 
     * @param nh ROS节点句柄
     * @param license_config 号牌配置参数
     */
    LicenseProcessor(ros::NodeHandle& nh,const LicenseConfig& license_config);
    
    /**
     * @brief 析构函数
     */
    ~LicenseProcessor();

public:
    /**
     * @brief 获取识别稳定的号牌信息
     * 
     * 返回已通过稳定性验证的号牌信息
     * 
     * @return LicenseInfo 稳定识别的号牌信息
     */
    LicenseInfo get_license_info_stable() const
    {
        return _license_info_stable;
    }
    
    /**
     * @brief 设置是否启动回调函数
     * 
     * @param flag 启动状态
     */
    void set_start(bool flag)
    { 
        _is_started = flag;
    } 
    
    /**
     * @brief 设置目标号牌
     * 
     * 设置需要识别和对准的目标号牌号码
     * 
     * @param license_no 目标号牌号码
     */
    void set_target_license(const std::string& license_no);
    
    /**
     * @brief 检查当前号牌是否在检测框内
     * 
     * 判断当前识别的号牌是否在检测框内
     * 
     * @return true 如果在检测框内
     * @return false 如果不在检测框内
     */
    bool is_in_detection_frame() const;

    /**
     * @brief 检查当前号牌信息是否有效
     * 
     * @return true 如果号牌信息有效
     * @return false 如果号牌信息无效
     */
    bool is_valid() const;
    
    /**
     * @brief 检查是否已对齐到号牌
     * 
     * 判断无人机是否已经对准到号牌的中心位置
     * 
     * @return true 如果已对齐
     * @return false 如果未对齐
     */
    bool is_aligned() const;
    
    /**
     * @brief 计算从当前号牌到目标位置的移动指令
     * 
     * 基于号牌在图像中的位置，计算无人机需要移动的方向和距离
     * 
     * @return std::tuple<char, double> 包含移动方向和距离的元组
     */
    std::tuple<char, double> calculate_movement_from_plate_diff() const;
    
    /**
     * @brief 计算指定轴的速度指令
     * 
     * 根据指定的轴向，计算相应的速度控制指令
     * 
     * @param axis 控制轴向
     * @return double 速度指令值
     */
    double calculate_velocity(const char& axis) const;
    
    /**
     * @brief 比较当前号牌与目标号牌
     * 
     * 检查当前识别的稳定号牌是否与目标号牌匹配
     * 
     * @return true 如果匹配
     * @return false 如果不匹配
     */
    bool compare_license() const;
    
    /**
     * @brief 添加拒绝号牌
     * 
     * 将当前识别的号牌添加到拒绝列表中
     */
    void add_rejected_plate();
    
    /**
     * @brief 清除已拒绝的号牌
     * 
     * 清空拒绝号牌列表
     */
    void clear_rejected_plates();

private:
    /** @brief ROS节点句柄引用 */
    ros::NodeHandle& _nh;
    /** @brief 处理器是否已启动 */
    bool _is_started = false;
    /** @brief 稳定识别的号牌信息 */
    LicenseInfo _license_info_stable;
    /** @brief 号牌配置参数 */
    LicenseConfig _license_config;
    /** @brief 号牌话题订阅者 */
    ros::Subscriber _license_sub;
    /** @brief 目标号牌号码 */
    std::string _target_license_no;
    /** @brief 最近一次识别到的号牌号码 */
    std::string _last_recognized_license_no;
    /** @brief 识别结果稳定性计数器（保留以保持兼容性） */
    int _stability_counter = 0;
    /** @brief 识别结果稳定性要求次数 */
    int _required_stability_count = 3;
    /** @brief 已拒绝的号牌集合 */
    std::set<std::string> _rejected_plates;
    /** @brief 号牌统计映射表
     * 
     * 记录每个识别到的号牌的统计信息
     */
    std::map<std::string, LicenseStats> _license_counters;

private:
    /**
     * @brief 号牌回调函数
     * 
     * 处理接收到的号牌检测消息
     * 
     * @param msg 号牌信息消息指针
     */
    void license_callback(const my_interfaces::LicenseInfo::ConstPtr& msg);
    
    /**
     * @brief 检查号牌信息是否超时
     * 
     * 判断从上次有效检测到现在的时间是否超过配置的超时时间
     * 
     * @return true 如果号牌信息已超时
     * @return false 如果号牌信息未超时
     */
    bool check_timeout(const LicenseInfo& license_info) const;
    
    /**
     * @brief 更新识别到的号牌信息
     * 
     * 处理新识别到的号牌信息，更新稳定性计数器
     * 
     * @param license_info 新的号牌信息
     */
    void update_recognized_license(const LicenseInfo& license_info);
    
    /**
     * @brief 检查号牌内容是否有效
     * 
     * 验证号牌号码格式是否符合要求
     * 
     * @param license_no 号牌号码
     * @return true 如果号牌号码有效
     * @return false 如果号牌号码无效
     */
    bool is_license_no_valid(const std::string& license_no) const;
};