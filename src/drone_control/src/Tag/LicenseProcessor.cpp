/**
 * @file LicenseProcessor.cpp
 * @brief 号牌处理器实现文件
 * 
 * 该文件实现了无人机号牌识别、跟踪和比对功能，包括号牌有效性验证、稳定性检测、
 * 移动方向计算和速度指令生成等，用于无人机的号牌确认任务。
 */
#include "Tag/LicenseProcessor.h"
#include <cmath>
#include <limits>

/**
 * @brief 构造函数实现
 * 
 * 初始化号牌处理器，订阅号牌识别话题
 * 
 * @param nh ROS节点句柄
 * @param license_config 号牌配置参数
 */
LicenseProcessor::LicenseProcessor(ros::NodeHandle& nh,const LicenseConfig& license_config)
    :_nh(nh), _license_config(license_config)
{
    // 订阅号牌识别话题
    _license_sub = _nh.subscribe(_license_config.license_topic, 1, &LicenseProcessor::license_callback, this);
}

/**
 * @brief 析构函数实现
 * 
 * 使用默认析构函数，订阅者会自动清理
 */
LicenseProcessor::~LicenseProcessor()=default;

/**
 * @brief 号牌信息回调函数
 * 
 * 处理从识别节点接收到的号牌信息，提取相关数据并更新状态
 * 
 * @param msg 包含号牌信息的ROS消息
 */
void LicenseProcessor::license_callback(const my_interfaces::LicenseInfo::ConstPtr& msg)
{
    // 检查是否已启动处理
    if(!_is_started) return;
    
    // 更新最新识别的号牌信息
    auto license_info = LicenseInfo{
        msg->license_no,
        msg->center_x,
        msg->center_y,
        msg->header.stamp
    };
    // 更新识别到的号牌信息
    update_recognized_license(license_info);
}

/**
 * @brief 更新识别到的号牌信息
 * 
 * 验证号牌有效性，更新稳定性计数器，并在达到稳定阈值时更新稳定的号牌信息
 * 
 * @param license_info 最新识别到的号牌信息
 */
void LicenseProcessor::update_recognized_license(const LicenseInfo& license_info)
{
    // 检查号牌内容是否有效
    if (!is_license_no_valid(license_info.license_no))
        return;
    
    // 清理过期的号牌统计数据
    auto now = ros::Time::now();
    for (auto it = _license_counters.begin(); it != _license_counters.end();) {
        if (now.toSec() - it->second.last_seen.toSec() > _license_config.target_timeout / 2.0) {
            it = _license_counters.erase(it);
        } else {
            ++it;
        }
    }
    
    // 更新当前号牌的计数器和最后出现时间
    std::string license_no = license_info.license_no;
    _license_counters[license_no].count++;
    _license_counters[license_no].last_seen = now;
    _license_counters[license_no].center_x = license_info.center_x;
    _license_counters[license_no].center_y = license_info.center_y;
    
    // 找出所有超过阈值且离图像中心点最近的号牌
    std::string closest_license_above_threshold;
    double min_distance = std::numeric_limits<double>::max();
    
    // 图像中心点坐标
    double image_center_x = _license_config.image_frame_size[0] / 2.0;
    double image_center_y = _license_config.image_frame_size[1] / 2.0;
    
    // 遍历所有号牌，找出符合阈值且离中心点最近的
    for (const auto& pair : _license_counters) {
        // 只考虑超过稳定性阈值的号牌
        if (pair.second.count >= _required_stability_count) {
            double current_center_x = pair.second.center_x;
            double current_center_y = pair.second.center_y;
            
            // 计算到图像中心点的欧几里得距离
            double distance = std::sqrt(
                std::pow(current_center_x - image_center_x, 2) + 
                std::pow(current_center_y - image_center_y, 2)
            );
            
            // 如果距离更近，则更新
            if (distance < min_distance) {
                min_distance = distance;
                closest_license_above_threshold = pair.first;
            }
        }
    }
    
    // 如果找到符合条件的号牌，更新稳定的号牌信息
    if (!closest_license_above_threshold.empty()) {
        _license_info_stable.license_no = closest_license_above_threshold;
        _license_info_stable.center_x = _license_counters[closest_license_above_threshold].center_x;
        _license_info_stable.center_y = _license_counters[closest_license_above_threshold].center_y;
        _license_info_stable.timestamp = now;
    }
    
}

/**
 * @brief 检查号牌信息是否超时
 * 
 * 比较当前时间与号牌信息时间戳，判断数据是否过期
 * 
 * @return 如果号牌信息超时返回true，否则返回false
 */
bool LicenseProcessor::check_timeout(const LicenseInfo& license_info) const
{
    return ros::Time::now().toSec() - license_info.timestamp.toSec() > _license_config.target_timeout;
}

/**
 * @brief 验证号牌号码格式是否有效
 * 
 * 检查号牌长度是否在有效范围内，并且后八位是否全部为数字
 * 
 * @param license_no 要验证的号牌号码
 * @return 如果号牌格式有效返回true，否则返回false
 */
bool LicenseProcessor::is_license_no_valid(const std::string& license_no) const
{
    // 检查号牌长度是否在有效范围内
    if (license_no.length() < 8 || license_no.length() > 13) 
        return false;
    
    // 检查后八位是否为数字
    std::string last_eight = license_no.substr(license_no.length() - 8, 8);
    bool all_digits = true;
    for (char c : last_eight) {
        if (!std::isdigit(static_cast<unsigned char>(c))) {
            all_digits = false;
            break;
        }
    }
    
    return all_digits;
}

/**
 * @brief 设置目标号牌号码
 * 
 * 指定要跟踪或寻找的目标号牌
 * 
 * @param license_no 目标号牌号码
 */
void LicenseProcessor::set_target_license(const std::string& license_no)
{
    _target_license_no = license_no;
}

/**
 * @brief 检查当前号牌是否在期望的检测框内
 * 
 * 验证当前识别的号牌中心是否在配置的检测框范围内
 * 
 * @return 如果号牌中心在检测框内返回true，否则返回false
 */
bool LicenseProcessor::is_in_detection_frame() const
{    
    // 计算检测框的边界
    double image_center_x = _license_config.image_frame_size[0] / 2.0;
    double image_center_y = _license_config.image_frame_size[1] / 2.0;
    
    double detection_half_width = _license_config.detection_frame_size[0] / 2.0;
    double detection_half_height = _license_config.detection_frame_size[1] / 2.0;
    
    // 检测框的左上角和右下角坐标
    double detection_left = image_center_x - detection_half_width;
    double detection_right = image_center_x + detection_half_width;
    double detection_top = image_center_y - detection_half_height;
    double detection_bottom = image_center_y + detection_half_height;
    
    // 检查号牌中心点是否在检测框内
    bool is_x_in_range = (_license_info_stable.center_x >= detection_left) && (_license_info_stable.center_x <= detection_right);
    bool is_y_in_range = (_license_info_stable.center_y >= detection_top) && (_license_info_stable.center_y <= detection_bottom);
    
    return is_x_in_range && is_y_in_range;
}

/**
 * @brief 检查当前稳定号牌信息是否有效
 * 
 * 验证当前稳定识别的号牌信息是否被更新、未超时且未被拒绝
 * 
 * @return 如果号牌信息有效返回true，否则返回false
 */
bool LicenseProcessor::is_valid() const
{
    // 检查是否为有效号牌、未超时且未被拒绝
    return _license_info_stable.is_valid() && !check_timeout(_license_info_stable) 
        && _rejected_plates.count(_license_info_stable.license_no) == 0;
}

/**
 * @brief 检查号牌中心是否在期望的检测框内
 * 
 * 判断号牌中心在X和Y方向上是否位于配置的检测框范围内
 * 
 * @return 如果号牌中心在检测框内返回true，否则返回false
 */
bool LicenseProcessor::is_aligned() const
{
    // 检查X方向对齐：判断号牌中心是否在X轴检测框范围内
    bool is_x_aligned = abs(_license_info_stable.center_x) >= (_license_config.image_frame_size[0]-_license_config.detection_frame_size[0])/2 
            and abs(_license_info_stable.center_x) <= (_license_config.image_frame_size[0]-_license_config.detection_frame_size[0])/2 + _license_config.detection_frame_size[0];
    
    // 检查Y方向对齐：判断号牌中心是否在Y轴检测框范围内
    bool is_y_aligned = abs(_license_info_stable.center_y) >= (_license_config.image_frame_size[1]-_license_config.detection_frame_size[1])/2 
            and abs(_license_info_stable.center_y) <= (_license_config.image_frame_size[1]-_license_config.detection_frame_size[1])/2 + _license_config.detection_frame_size[1];
    
    // 只有X和Y方向都对齐时才返回true
    return is_x_aligned and is_y_aligned;
}

/**
 * @brief 根据号牌编号差异计算移动方向
 * 
 * 解析目标号牌和检测到的号牌，计算垂直（Z轴）和水平（Y轴）方向上的移动需求
 * 
 * @return 包含移动轴('y'/'z')和方向(正负值)的元组，无移动时返回('\0', 0.0)
 */
std::tuple<char, double> LicenseProcessor::calculate_movement_from_plate_diff() const
{
    // 提取后四位的前两位（控制Z轴，不同层）
    int target_layer = std::stoi(_target_license_no.substr(_target_license_no.length() - 4, 2));
    int detected_layer = std::stoi(_license_info_stable.license_no.substr(_license_info_stable.license_no.length() - 4, 2));
    
    // 提取后两位数字（控制Y轴，同一层不同位置）
    int target_last_two = std::stoi(_target_license_no.substr(_target_license_no.length() - 2, 2));
    int detected_last_two = std::stoi(_license_info_stable.license_no.substr(_license_info_stable.license_no.length() - 2, 2));
    
    // 计算层差异（Z轴）
    int layer_diff = target_layer - detected_layer;
    
    // 优先处理Z轴方向（垂直方向，不同层）
    if (layer_diff != 0) {
        // 目标层大于当前层时向上飞（正值），目标层小于当前层时向下飞（负值）
        double direction = (layer_diff > 0) ? 1.0 : -1.0;
        return std::make_tuple('z', direction);
    }
    
    // Z轴无差异时，处理Y轴方向（水平方向，同一层不同位置）
    int position_diff = target_last_two - detected_last_two;
    
    if (position_diff != 0) {
        // 目标位置大于当前位置时向右飞（负值），目标位置小于当前位置时向左飞（正值）
        double direction = (position_diff > 0) ? -1.0 : 1.0;
        return std::make_tuple('y', direction);
    }
    
    // 默认返回，当不需要移动时
    return std::make_tuple('\0', 0.0);
}

/**
 * @brief 计算指定轴的速度指令
 * 
 * 根据号牌中心位置与检测框的关系，计算机体坐标系下的速度指令
 * 
 * @param axis 要计算速度的轴，'y'或'z'
 * @return 对应轴的速度指令值
 */
double LicenseProcessor::calculate_velocity(const char& axis) const
{
    /*
    计算基于机体坐标系的速度指令，使目标号牌中心点在检测框边界内
    前提是机头正对号牌，容忍一些角度误差
    机体坐标系：
    - Y轴：指向左侧
    - Z轴：指向顶部
    像素坐标系：
    - X轴：指向右侧
    - Y轴：指向底部
    */
    // 检查X方向对齐
    if(!_license_info_stable.is_valid())
    {
        return 0.0;
    }
    bool is_x_aligned = abs(_license_info_stable.center_x) >= (_license_config.image_frame_size[0]-_license_config.detection_frame_size[0])/2 
            and abs(_license_info_stable.center_x) <= (_license_config.image_frame_size[0]-_license_config.detection_frame_size[0])/2 + _license_config.detection_frame_size[0];
    
    // 检查Y方向对齐
    bool is_y_aligned = abs(_license_info_stable.center_y) >= (_license_config.image_frame_size[1]-_license_config.detection_frame_size[1])/2 
            and abs(_license_info_stable.center_y) <= (_license_config.image_frame_size[1]-_license_config.detection_frame_size[1])/2 + _license_config.detection_frame_size[1];
    
    // 计算Y轴速度
    if (axis == 'y')
    {
        if (is_x_aligned) 
        {
            return 0.0;
        }
        // 未对齐时，根据中心位置判断移动方向
        else 
        {
            if (abs(_license_info_stable.center_x)<(_license_config.image_frame_size[0]-_license_config.detection_frame_size[0])/2) 
            {
                return -0.1;
            } else 
            {
                return 0.1;
            }
        }
    }
    
    // 计算Z轴速度
    if (axis == 'z')
    {
        if (is_y_aligned) 
        {
            return 0.0;
        }
        // 未对齐时，根据中心位置判断移动方向
        else 
        {
            if (abs(_license_info_stable.center_y)<(_license_config.image_frame_size[1]-_license_config.detection_frame_size[1])/2) 
            {
                return -0.1;
            } else 
            {
                return 0.1;
            }
        }
    }
    
    // 默认返回，当不需要移动时
    return 0.0;
}

/**
 * @brief 比较当前稳定的号牌是否匹配目标号牌
 * 
 * @return 如果号牌匹配返回true，否则返回false
 */
bool LicenseProcessor::compare_license() const
{
    return _license_info_stable.license_no == _target_license_no;
}

/**
 * @brief 添加当前稳定的号牌到拒绝列表
 * 
 * 当确认当前号牌不是目标号牌时，将其添加到拒绝列表，避免重复检查
 */
void LicenseProcessor::add_rejected_plate()
{
    _rejected_plates.insert(_license_info_stable.license_no);
}

/**
 * @brief 清除所有被拒绝的号牌
 * 
 * 重置拒绝列表，允许重新检查之前被拒绝的号牌
 */
void LicenseProcessor::clear_rejected_plates()
{
    _rejected_plates.clear();
}