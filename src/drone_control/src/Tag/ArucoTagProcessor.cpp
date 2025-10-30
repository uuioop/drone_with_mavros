/**
 * @file ArucoTagProcessor.cpp
 * @brief ArUco标记处理器实现文件
 * 
 * 该文件实现了ArUco标记的检测、坐标系转换、姿态估计和速度控制指令计算等功能，
 * 用于无人机的视觉定位和精确导航。
 */
#include <unordered_map>
#include <fiducial_msgs/FiducialTransformArray.h>
#include "Tag/ArucoTagProcessor.h"
#include "util/utils.h"

/**
 * @brief 构造函数实现
 * 
 * 初始化处理器，订阅标记检测话题
 * 
 * @param nh ROS节点句柄
 * @param tag_config 标记配置参数
 */
ArucoTagProcessor::ArucoTagProcessor(ros::NodeHandle& nh, const TagConfig tag_config):
    _nh(nh),_tag_config(tag_config)
{
   // 订阅标记检测话题
   _target_pose_sub = _nh.subscribe(_tag_config.topic_name, 1, &ArucoTagProcessor::target_pose_callback, this);
}

/**
 * @brief 析构函数实现
 * 
 * 使用默认析构函数，订阅者会自动清理
 */
ArucoTagProcessor::~ArucoTagProcessor()=default;

/**
 * @brief 目标位姿回调函数
 * 
 * 处理从相机接收到的标记位姿信息，将其转换为机身坐标系
 * 
 * @param msg 包含标记位姿的ROS消息
 */
void ArucoTagProcessor::target_pose_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
    // 检查是否已启动处理
    // if(!_is_started)
    //     return;
    ROS_INFO_ONCE("ArucoTagProcessor: 收到标记位姿");
    // 检查是否有检测到的标记
    if(msg->transforms.empty()) {
        return;
    }

    // 遍历所有transforms，选择特定ID的标记
    for(const auto& transform : msg->transforms) {
        // 选择特定ID的标记
        if(transform.fiducial_id != _tag_config.target_id)
            continue;
        
        // 从transform中提取标记位姿信息
        auto tag = ArucoTag {
            Eigen::Vector3d(transform.transform.translation.x, 
                           transform.transform.translation.y, 
                           transform.transform.translation.z),
            Eigen::Quaterniond(transform.transform.rotation.w, 
                              transform.transform.rotation.x, 
                              transform.transform.rotation.y, 
                              transform.transform.rotation.z),
            msg->header.stamp
        };
        if (_tag_config.target_id == 0)
            ROS_INFO("%s position: %f, %f, %f", _tag_config.tag_name.c_str(), tag.position.x(), tag.position.y(), tag.position.z());
        // 将相机坐标系下的标记位姿转换为机身坐标系
        _tag = get_tag_body(tag);
        break;  // 只处理第一个标记
    }
}

/**
 * @brief 将相机坐标系下的标记位姿转换为机身坐标系
 * 
 * 应用配置中定义的旋转矩阵进行坐标系转换
 * 
 * @param tag_cam 相机坐标系下的标记位姿
 * @return 机身坐标系下的标记位姿
 */
ArucoTag ArucoTagProcessor::get_tag_body(const ArucoTag& tag_cam) const
{
    // mavros的接口是FLU而非FRD坐标系(不同于PX4)，按ros的标准
    // 检查四元数有效性
    if (tag_cam.orientation.coeffs().norm() < 1e-6)   // coeffs() = (x,y,z,w)
    {
        if (tag_cam.is_valid())          // 从有效变无效才打印
            ROS_WARN("[TagProcessor] 收到一个范数为零的无效四元数，已忽略。");
        return ArucoTag();                   // 重置为无效（timestamp=0）
    }
    
    // 使用旋转矩阵创建四元数
    Eigen::Quaterniond quat_cam_to_body(_tag_config.rotation_matrix);
    
    // 位置变换：将相机坐标系下的位置转换到机身坐标系
    Eigen::Vector3d tag_body_position = quat_cam_to_body * tag_cam.position;
    
    // 姿态变换：将相机坐标系下的姿态转换到机身坐标系
    Eigen::Quaterniond tag_body_orientation = quat_cam_to_body * tag_cam.orientation;
    
    // 确保四元数归一化
    tag_body_orientation.normalize();
   
    // 返回转换后的标记位姿
    return ArucoTag{
        tag_body_position,
        tag_body_orientation,
        tag_cam.timestamp
    };
}

/**
 * @brief 检查标记是否超时
 * 
 * 比较当前时间与标记时间戳，判断标记数据是否过期
 * 
 * @return 如果标记超时返回true，否则返回false
 */
bool ArucoTagProcessor::check_tag_timeout() const
{
    // 检查标记是否超时
    return (ros::Time::now() - _tag.timestamp).toSec() > _tag_config.target_timeout;
}

/**
 * @brief 检查标记与目标的距离是否在有效范围内
 * 
 * 验证标记在X、Y、Z轴上的距离是否小于配置的最大检测距离
 * 
 * @return 如果距离有效返回true，否则返回false
 */
bool ArucoTagProcessor::check_distance_valid() const
{
    if (!_tag.is_valid() && check_tag_timeout()) {
        return false;
    }

    // 检查标记与目标的距离是否在有效范围内
    bool x_valid= std::abs(_tag.position.x()) < _tag_config.max_detection_distance[0];
    bool y_valid= std::abs(_tag.position.y()) < _tag_config.max_detection_distance[1];
    bool z_valid= std::abs(_tag.position.z()) < _tag_config.max_detection_distance[2];
    return x_valid and y_valid and z_valid;
}

/**
 * @brief 获取偏航角误差（角度）
 * 
 * 根据标记的朝向计算相对于期望方向的偏航角误差
 * 
 * @return 偏航角误差（角度制）
 */
double ArucoTagProcessor::get_yaw_error_deg() const
{
    // 使用四元数变换向量 [0, 0, -1]，得到mark_x方向向量
    Eigen::Vector3d mark_dir(0.0, 0.0, -1.0);
    Eigen::Vector3d mark_x = _tag.orientation * mark_dir;
    
    // 计算偏航角误差（弧度）
    double yaw_error_rad = std::atan2(mark_x.y(), mark_x.x());
    
    // 转换为角度并归一化（通过rad2deg_normalized实现）
    return rad2deg_normalized(yaw_error_rad);
}

/**
 * @brief 计算偏航角速度指令
 * 
 * 使用P控制器计算偏航角速度，并应用滤波和平滑处理，确保角速度在有效范围内
 * 
 * @return 偏航角速度指令（角度/秒）
 */
double ArucoTagProcessor::calculate_yaw_rate() const
{
    // 从配置中获取控制参数
    double kp = _tag_config.kp;
    double max_yaw_rate_deg_s = _tag_config.max_yaw_rate_deg_s;
    double min_yaw_rate_deg_s = _tag_config.min_yaw_rate_deg_s;
    double yaw_tolerance_deg = _tag_config.yaw_tolerance_deg;
    double filter_alpha = _filter_alpha;
    
    // 获取当前偏航角误差
    double yaw_error_deg = get_yaw_error_deg();
    
    // 更新滤波后的误差（使用mutable关键字允许在const函数中修改）
    _filtered_yaw_error_deg = (filter_alpha * yaw_error_deg) + 
                             (1.0 - filter_alpha) * _filtered_yaw_error_deg;
    
    // 使用滤波后的平滑值进行后续所有计算，确保误差范围在[-180, 180]度之间
    double yaw_diff = (_filtered_yaw_error_deg + 180.0) - 
                     std::floor((_filtered_yaw_error_deg + 180.0) / 360.0) * 360.0 - 180.0;
    
    // 检查是否在容差范围内，在容差范围内不产生控制指令
    if (std::abs(yaw_diff) <= yaw_tolerance_deg) {
        return 0.0;
    }
    
    // 计算期望的偏航角速度（P控制）
    double desired_yaw_rate = kp * yaw_diff;
    double applied_yaw_rate = desired_yaw_rate;
    
    // 根据最大/最小限制调整应用的偏航角速度
    if (std::abs(desired_yaw_rate) > max_yaw_rate_deg_s) {
        applied_yaw_rate = std::copysign(max_yaw_rate_deg_s, desired_yaw_rate);
    } else if (std::abs(desired_yaw_rate) < min_yaw_rate_deg_s) {
        applied_yaw_rate = std::copysign(min_yaw_rate_deg_s, desired_yaw_rate);
    }
    
    return applied_yaw_rate;
}

/**
 * @brief 更新标记有效性状态并记录日志
 * 
 * 检查标记是否有效、是否超时、距离是否有效，并在状态变化时记录ROS日志
 */
void ArucoTagProcessor::update_and_log_validity()
{
    std::string tag_name=_tag_config.tag_name;
    
    // 综合评估标记有效性
    _is_valid_now=_tag.is_valid() and !check_tag_timeout() and check_distance_valid();
    
    // 记录状态变化
    if (!_is_valid_now and _was_previously_valid) 
        ROS_WARN("%s 丢失", tag_name.c_str());
    else if (_is_valid_now and !_was_previously_valid)
        ROS_INFO("%s 重新获取", tag_name.c_str());

    // 更新前一时刻的有效性状态
    _was_previously_valid=_is_valid_now;
}

/**
 * @brief 检查标记是否已对齐
 * 
 * 根据配置的对齐轴和容差，判断标记是否在期望的位置范围内，并使用计时器确保持续对齐
 * 
 * @return 如果标记持续对齐返回true，否则返回false
 */
bool ArucoTagProcessor::is_aligned() const
{
    // 首先检查标记是否有效
    if (!_is_valid_now) {
        return false;
    }
    
    const auto& alignment_axes = _tag_config.alignment_axes;
    const double aligned_tolerance = _tag_config.aligned_tolerance;
    
    // 创建误差向量
    Eigen::VectorXd error_vector;
    int dim_count = 0;
    
    // 根据alignment_axes中包含的坐标轴添加对应的位置误差
    for (const auto& axis : alignment_axes) {
        if (axis == 'x' || axis == 'y' || axis == 'z') {
            dim_count++;
        }
    }
    
    // 如果dim_count为0，无法进行对齐检查，重置计时器并返回false
    if (dim_count == 0) {
        ROS_ERROR_ONCE("%s: 对齐轴配置为空，无法进行对齐检查", _tag_config.tag_name.c_str());
        _align_timer = 0;
        return false;
    }
    
    // 初始化误差向量
    error_vector.resize(dim_count);
    int idx = 0;
    
    // 填充误差向量，根据配置的对齐轴收集对应轴的位置误差
    for (const auto& axis : alignment_axes) {
        if (axis == 'x') {
            error_vector(idx++) = _tag.position.x();
        } else if (axis == 'y') {
            error_vector(idx++) = _tag.position.y();
        } else if (axis == 'z') {
            error_vector(idx++) = _tag.position.z();
        }
    }
    
    // 计算误差向量的范数并与容差比较
    if (error_vector.norm() < aligned_tolerance) {
        _align_timer++;
    } else {
        _align_timer = 0;
    }
    
    // 检查是否持续对准15次（防止噪声导致的误判）
    return _align_timer >= 15;
}

/**
 * @brief 获取标记对齐误差
 * 
 * 计算标记在X、Y、Z轴上的位置误差以及偏航角误差
 * 
 * @return 包含[x_error, y_error, z_error, yaw_error_deg]的数组
 * @throws std::runtime_error 如果标记无效
 */
std::array<double,4> ArucoTagProcessor::get_alignment_errors() const
{
    // 检查标签是否有效且未超时
    if (_is_valid_now)
    {
        // 获取目标偏移量
        Eigen::Vector3d target_offset = _tag_config.target_offset;

        // 获取偏航角误差（角度）
        double yaw_error_deg = get_yaw_error_deg();
        
        // 返回包含[x_error, y_error, z_error, yaw_error_deg]的数组
        return {
            _tag.position(0) - target_offset(0),
            _tag.position(1) - target_offset(1),
            _tag.position(2) - target_offset(2),
            yaw_error_deg
        };
    }
    else
        throw std::runtime_error("Tag is not valid");
}

/**
 * @brief 计算速度控制指令
 * 
 * 使用P控制器根据标记位置误差计算速度指令，并应用速度限制
 * 
 * @param is_yaw 是否启用偏航控制
 * @return 包含[vx, vy, vz, yaw_rate]的速度指令数组
 */
std::array<double,4> ArucoTagProcessor::calculate_velocity_command(bool is_yaw) const
{
    // 检查标签是否有效且未超时
    if (!_is_valid_now) {
        return {0.0, 0.0, 0.0, 0.0};
    }
    
    // 从配置中获取控制参数
    double kp = _tag_config.kp;
    double max_speed = _tag_config.max_speed;
    Eigen::Vector3d target_offset=_tag_config.target_offset;
    
    // 计算误差：当前位置 - 期望的目标位置
    Eigen::Vector3d error = _tag.position - target_offset;
    
    // 使用P控制计算速度指令
    Eigen::Vector3d velocity;
    velocity(0) = kp * error(0); // vx
    velocity(1) = kp * error(1); // vy
    velocity(2) = kp * error(2); // vz
    
    // 对速度进行限幅，确保不超过最大速度限制
    velocity = velocity.cwiseMax(-max_speed).cwiseMin(max_speed);
    
    // 如果需要偏航控制，使用calculate_yaw_rate方法
    double yaw_rate = 0.0;
    if (is_yaw) 
        yaw_rate = calculate_yaw_rate();
    
    // 返回完整的速度指令数组
    return {velocity(0), velocity(1), velocity(2), yaw_rate};
}
