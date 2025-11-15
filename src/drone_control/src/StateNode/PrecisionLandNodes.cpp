/**
 * @file PrecisionLandNodes.cpp
 * @brief 精确着陆状态节点实现文件
 * 
 * 该文件实现了精确着陆控制器中的各个状态节点，包括搜索标记、对准引导标记、
 * 接近引导标记、对准平台标记、下降和着陆六个状态。这些状态节点用于控制无人机
 * 完成精确着陆任务。
 */

#include <mavros_msgs/ExtendedState.h>

#include "StateNode/PrecisionLandNodes.h" 
#include "MissionNode/PrecisionLandMission.h"

/**
 * @brief SearchTagState构造函数
 * 
 * 初始化搜索标记状态节点，获取前端标记处理器并设置超时时长
 * 
 * @param mission 所在任务引用
 */
SearchTagState::SearchTagState(MissionNode& mission):StateNode(mission)
{
    // 获取前端标记处理器(转换为PrecisionLandMission类型)
    _front_tag_processor=static_cast<PrecisionLandMission&>(_mission).get_front_tag_processor();
}

/**
 * @brief 进入搜索标记状态时调用
 * 
 * 记录日志并设置进入时间
 */
void SearchTagState::on_enter()
{
    ROS_INFO("[PL] 进入：搜索状态");
    _entry_time= ros::Time::now();
}

/**
 * @brief 搜索标记状态更新方法
 * 
 * 检查是否发现引导标记，超时则直接降落，否则保持上升
 * 
 * @return 速度控制指令数组 [v_x, v_y, v_z, v_yaw]
 */
std::array<double,4> SearchTagState::on_update()
{
    // 检查是否发现引导标记
    if(_front_tag_processor->is_valid())
    {
        ROS_INFO("[PL] 发现引导标记，开始对准...");
        static_cast<PrecisionLandMission&>(_mission).switch_node("align_on_guide_tag");
        return {0.0,0.0,0.0,0.0};
    }
    // 检查是否超时
    if(is_timeout())
    {
        // 超时，降落
        ROS_WARN("[PL] 搜索状态超时，降落");
        static_cast<PrecisionLandMission&>(_mission).switch_node("land");
        _vel_cmd={0.0,0.0,0.0,0.0};
    }
    else
        // 上升
        _vel_cmd={0.0,0.0,0.5,0.0};
    return _vel_cmd;
}

/**
 * @brief 退出搜索标记状态时调用
 * 
 * 目前为空实现
 */
void SearchTagState::on_exit()
{
    // 退出搜索状态
}

/**
 * @brief AlignOnGuideTagState构造函数
 * 
 * 初始化对准引导标记状态节点，获取前端标记处理器并设置对准容差
 * 
 * @param mission 所在任务引用
 */
AlignOnGuideTagState::AlignOnGuideTagState(MissionNode& mission):StateNode(mission)
{
    // 获取前端标记处理器(转换为PrecisionLandMission类型)
    _front_tag_processor=static_cast<PrecisionLandMission&>(_mission).get_front_tag_processor();
    // 设置对准容差
    _align_tolerance={
        {"yaw",20.0},
        {"z",0.4},
        {"x",6.0}
    };
}

/**
 * @brief 进入对准引导标记状态时调用
 * 
 * 记录日志
 */
void AlignOnGuideTagState::on_enter()
{
    ROS_INFO("[PL] 进入：对准引导标记状态");
}

/**
 * @brief 对准引导标记状态更新方法
 * 
 * 根据标记处理器计算的误差向量，按优先级调整无人机姿态和位置，
 * 对准完成后切换到接近引导标记状态
 * 
 * @return 速度控制指令数组 [v_x, v_y, v_z, v_yaw]
 */
std::array<double,4> AlignOnGuideTagState::on_update()
{
    if (!_front_tag_processor->is_valid())
    {
        ROS_WARN("[PL] 对准前置引导标记状态下，搜索不到引导标记，等待后续命令");
        return {0.0, 0.0, 0.0, 0.0};
    }
    // 检查是否对准完成
    if(_front_tag_processor->is_aligned())
        static_cast<PrecisionLandMission&>(_mission).switch_node("approach_guide_tag");

    try {
        _alignment_errors=_front_tag_processor->get_alignment_errors();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("[PL] 对准前置引导标记状态, 获取误差向量失败: %s", e.what());
        return {0.0, 0.0, 0.0, 0.0};
    }
    _vel_cmd=_front_tag_processor->calculate_velocity_command(true);
    // 如果偏航角误差较大，则优先调整角度，限制平移速度
    if (std::abs(_alignment_errors[3])>_align_tolerance["yaw"])
    { 
        _vel_cmd[0]=0.0;
        _vel_cmd[1]=0.0;
        // 使用std::min和std::max实现类似np.clip的功能，将vz限制在[-0.2, 0.2]
        _vel_cmd[2] = std::max(-0.2, std::min(_vel_cmd[2], 0.2));
    }
    // 如果Z轴误差较大，优先调整高度，限制其他轴速度
    else if (std::abs(_alignment_errors[2])>_align_tolerance["z"])
    {
        _vel_cmd[0]=0.0;
        _vel_cmd[1]=std::max(-0.2, std::min(_vel_cmd[1], 0.2));
    }
    // 如果X轴误差较大，允许小范围移动
    if (std::abs(_alignment_errors[0])>_align_tolerance["x"])
        _vel_cmd[0]=std::max(-0.15, std::min(_vel_cmd[0], 0.15));
    
    return _vel_cmd;
}

/**
 * @brief 退出对准引导标记状态时调用
 * 
 * 目前为空实现
 */
void AlignOnGuideTagState::on_exit()
{
    // 退出对准引导标记状态
}

/**
 * @brief ApproachGuideTagState构造函数
 * 
 * 初始化接近引导标记状态节点，获取前端和下降标记处理器
 * 
 * @param mission 所在任务引用
 */
ApproachGuideTagState::ApproachGuideTagState(MissionNode& mission):StateNode(mission)
{
    // 获取前端和下降标记处理器(转换为PrecisionLandMission类型)
    _front_tag_processor=static_cast<PrecisionLandMission&>(_mission).get_front_tag_processor();
    _down_tag_processor=static_cast<PrecisionLandMission&>(_mission).get_down_tag_processor();
}

/**
 * @brief 进入接近引导标记状态时调用
 * 
 * 记录日志
 */
void ApproachGuideTagState::on_enter()
{
    ROS_INFO("[PL] 进入：接近引导标记状态");
}

/**
 * @brief 接近引导标记状态更新方法
 * 
 * 保持对准引导标记前进，当检测到下方平台标记时切换到对准平台标记状态
 * 
 * @return 速度控制指令数组 [v_x, v_y, v_z, v_yaw]
 */
std::array<double,4> ApproachGuideTagState::on_update()
{
    if (!_front_tag_processor->is_valid())
    {
        ROS_WARN("[PL] 接近引导标记状态下，搜索不到引导标记，等待后续命令");
        return {0.0, 0.0, 0.0, 0.0};
    }
    if (_down_tag_processor->is_valid())
    {
        ROS_INFO("[PL] 接近引导标记状态下，搜索到下降标记，开始下降...");
        static_cast<PrecisionLandMission&>(_mission).switch_node("descend");
        return {0.0,0.0,0.0,0.0};
    }
    return _front_tag_processor->calculate_velocity_command(true);
}

/**
 * @brief 退出接近引导标记状态时调用
 * 
 * 目前为空实现
 */
void ApproachGuideTagState::on_exit()
{
    // 退出接近引导标记状态
}

/**
 * @brief AlignOnPlatformTagState构造函数
 * 
 * 初始化对准平台标记状态节点，获取下降标记处理器
 * 
 * @param mission 所在任务引用
 */
AlignOnPlatformTagState::AlignOnPlatformTagState(MissionNode& mission):StateNode(mission)
{
    // 获取下降标记处理器(转换为PrecisionLandMission类型)
    _down_tag_processor=static_cast<PrecisionLandMission&>(_mission).get_down_tag_processor();
}

/**
 * @brief 进入对准平台标记状态时调用
 * 
 * 记录日志
 */
void AlignOnPlatformTagState::on_enter()
{
    ROS_INFO("[PL] 进入：对准平台标记状态");
}

/**
 * @brief 对准平台标记状态更新方法
 * 
 * 调整无人机位置以精确对准平台标记，对准完成后切换到下降状态
 * 
 * @return 速度控制指令数组 [v_x, v_y, v_z, v_yaw]
 */
std::array<double,4> AlignOnPlatformTagState::on_update()
{
    if (!_down_tag_processor->is_valid())
    {
        ROS_WARN("[PL] 对准平台标记状态下，搜索不到平台标记，等待后续命令");
        return {0.0, 0.0, 0.0, 0.0};
    }
    // 检查是否对准完成
    if(_down_tag_processor->is_aligned())
    {
        static_cast<PrecisionLandMission&>(_mission).switch_node("descend");
        return {0.0, 0.0, 0.0, 0.0};
    }

    // 不使用偏航角误差
    _vel_cmd=_down_tag_processor->calculate_velocity_command(false);
    // 禁用Z轴速度
    _vel_cmd[2]=0.0;
    return _vel_cmd;
}

/**
 * @brief 退出对准平台标记状态时调用
 * 
 * 目前为空实现
 */
void AlignOnPlatformTagState::on_exit()
{
    // 退出对准平台标记状态
}

/**
 * @brief 下降状态构造函数
 * 
 * 初始化下降状态节点，获取下降标记处理器
 * 
 * @param mission 所在任务引用
 */
DescendState::DescendState(MissionNode& mission):StateNode(mission)
{
    _timeout_duration=ros::Duration(1.5);
    // 获取下降标记处理器(转换为PrecisionLandMission类型)
    _down_tag_processor=static_cast<PrecisionLandMission&>(_mission).get_down_tag_processor();
}

/**
 * @brief 进入下降状态
 * 
 * 记录进入下降状态的日志信息
 */
void DescendState::on_enter()
{
    ROS_INFO("[PL] 进入：下降状态");
    _entry_time=ros::Time::now();
}

/**
 * @brief 更新下降状态
 * 
 * 处理下降逻辑，包括平台标记检测、着陆状态检查和速度计算
 * @return 速度指令数组 [vx, vy, vz, yaw_rate]
 */
std::array<double,4> DescendState::on_update()
{

    // 检查是否下降完成
    if(_mission.get_status_monitor().get_landed_state() ==
        mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {  
        _mission.set_finished(true);
        return {0.0, 0.0, 0.0, 0.0};
    }
    
    if (!_down_tag_processor->is_valid())
    {    
        if(is_timeout())
            static_cast<PrecisionLandMission&>(_mission).switch_node("land");
        return {0.0, 0.0, -0.1, 0.0};
    }
    // 平台标记有效时重置进入时间
    _entry_time=ros::Time::now();

    // 计算下降速度
    _vel_cmd=_down_tag_processor->calculate_velocity_command(false);
    try {
        _alignment_errors=_down_tag_processor->get_alignment_errors();
    } catch (const std::runtime_error& e) {
        ROS_ERROR("[PL] 下降状态下，获取平台标记误差向量失败: %s", e.what());
        return {0.0, 0.0, 0.0, 0.0};
    }
    if (_down_tag_processor->is_aligned())
        _vel_cmd[2]=-0.3;
    else
        _vel_cmd[2]=0.0;
    // ROS_INFO("vel:%lf,%lf",_vel_cmd[0],_vel_cmd[1]);
    // ROS_INFO("error:%lf,%lf,%lf",_alignment_errors[0],_alignment_errors[1],_alignment_errors[2]);
    return _vel_cmd;
}

/**
 * @brief 退出下降状态
 * 
 * 退出下降状态时调用，目前为空实现
 */
void DescendState::on_exit()
{
    // 退出下降状态
}

/**
 * @brief 进入着陆状态
 * 
 * 记录进入着陆状态的日志信息
 */
void LandState::on_enter()
{
    ROS_INFO("[PL] 进入：着陆状态");
}

/**
 * @brief 更新着陆状态
 * 
 * 处理着陆逻辑，检查着陆状态并返回固定的下降速度
 * @return 速度指令数组 [vx, vy, vz, yaw_rate]
 */
std::array<double,4> LandState::on_update()
{
    // 检查是否下降完成
    if(_mission.get_status_monitor().get_landed_state() ==
        mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {  
        _mission.set_finished(true);
        return {0.0, 0.0, 0.0, 0.0};
    }
    return {0.0, 0.0, -0.5, 0.0};
}

/**
 * @brief 退出着陆状态
 * 
 * 退出着陆状态时调用，目前为空实现
 */
void LandState::on_exit()
{
    // 退出着陆状态
}
