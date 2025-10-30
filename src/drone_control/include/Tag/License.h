#pragma once

#include <array>
#include <string>
#include <ros/ros.h>

/**
 * @file License.h
 * @brief 号牌相关的数据结构定义
 * 
 * 该文件定义了用于表示号牌信息的结构体，
 * 以及号牌检测和处理所需的配置参数结构体。
 */

/**
 * @brief 号牌信息结构体
 * 
 * 用于存储检测到的号牌信息，包括号牌号码、位置和时间戳。
 */
struct LicenseInfo{
    /** @brief 号牌号码字符串 */
    std::string license_no="";
    /** @brief 号牌在图像中的x坐标中心位置 */
    double center_x=0.0;
    /** @brief 号牌在图像中的y坐标中心位置 */
    double center_y=0.0;
    /** @brief 号牌检测的时间戳 */
    ros::Time timestamp=ros::Time(0);
    
    /**
     * @brief 检查号牌数据是否有效
     * 
     * 通过判断时间戳是否大于0来确定数据是否已被更新
     * 
     * @return true 如果号牌数据有效
     * @return false 如果号牌数据无效
     */
    bool is_valid() const
    {
        return timestamp.toSec() > 0.0;
    }
};

/**
 * @brief 号牌配置结构体
 * 
 * 用于存储号牌检测和处理的配置参数。
 */
struct LicenseConfig{
    /** @brief 号牌检测结果话题名称 */
    std::string license_topic="/license_detection_result";
    /** @brief 目标超时时间，单位：秒 */
    double target_timeout=1.0;
    /** @brief 图像大小 [宽度, 高度] */
    std::array<int,2> image_frame_size={1920,1080};
    /** @brief 检测框大小 [宽度, 高度]，为图像大小的三分之一 */
    std::array<int,2> detection_frame_size={640,360};
};
