/**
 * @file utils.h
 * @brief 数学工具函数头文件
 * 
 * 该文件提供了角度和弧度转换相关的工具函数，用于无人机控制中的坐标变换和姿态计算。
 */
#pragma once
#include <cmath>
#include <array>


/**
* @brief 将角度转换为弧度并归一化到[-π, π]范围
* @param degrees 角度值
* @return 归一化后的弧度值
*/
inline double deg2rad_normalized(double degrees) {
    // 首先，将角度归一化到[-180, 180)范围
    double deg_norm = std::fmod(degrees + 180.0, 360.0);
    if (deg_norm < 0) {
        deg_norm += 360.0;
    }
    deg_norm -= 180.0; // 范围变为 [-180, 180)

    // 然后转换为弧度
    return deg_norm * M_PI / 180.0;
}

/**
* @brief 将弧度转换为角度并归一化到[-180, 180]范围
* @param radians 弧度值
* @return 归一化后的角度值
*/
inline double rad2deg_normalized(double radians) {
    // 首先，将弧度归一化到[-π, π)范围
    // atan2(sin(x), cos(x)) 是一个非常稳定和健壮的方法来获取x的等效主值
    double rad_norm = std::atan2(std::sin(radians), std::cos(radians));
    
    // 然后转换为角度
    return rad_norm * 180.0 / M_PI;
}

/**
 * @brief 检查目标位置数据的有效性
 * @param lat 纬度(度)
 * @param lon 经度(度)
 * @param alt 相对高度(米)
 * @return 位置数据有效返回true
 * @note 检测标准：
 *   - 经纬度为非NaN且在合理范围内(-90到90度，-180到180度)
 *   - 相对高度为非NaN且不小于-0.1米
 */
inline bool check_target_pos_valid(std::array<double, 3> target_pos){
    // 检查是否为NaN
    if (std::isnan(target_pos[0]) || std::isnan(target_pos[1]) || std::isnan(target_pos[2])) {
        return false;
    }
    
    // 检查纬度范围：-90到90度
    if (target_pos[0] < -90.0 || target_pos[0] > 90.0) {
        return false;
    }
    
    // 检查经度范围：-180到180度
    if (target_pos[1] < -180.0 || target_pos[1] > 180.0) {  
        return false;
    }
    
    // 检查相对高度：不小于-0.1米
    if (target_pos[2] < -0.1) {
        return false;
    }
    
    return true;
}