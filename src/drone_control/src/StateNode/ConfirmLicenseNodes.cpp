/**
 * @file ConfirmLicenseNodes.cpp
 * @brief 号牌确认状态节点实现文件
 * 
 * 该文件实现了号牌确认控制器中的各个状态节点，包括搜索号牌、确认号牌和重新定位号牌。
 * 这些状态节点用于控制无人机在号牌确认过程中的行为。
 */

#include "StateNode/ConfirmLicenseNodes.h"
#include "MissionNode/ConfirmLicenseMission.h"

/**
 * @brief SearchLicenseState构造函数
 * 
 * 初始化搜索号牌状态节点，获取号牌处理器引用
 * 
 * @param mission 所在任务引用
 */
SearchLicenseState::SearchLicenseState(MissionNode& mission):StateNode(mission)
{
    // 获取号牌处理器
    _license_processor=static_cast<ConfirmLicenseMission&>(_mission).get_license_processor();
}

/**
 * @brief 进入搜索号牌状态时调用
 * 
 * 记录日志并获取移动向量
 */
void SearchLicenseState::on_enter()
{
    ROS_INFO("[CL] SearchLicenseState::on_enter");
    _move_vector=static_cast<ConfirmLicenseMission&>(_mission).get_move_vector();
}

/**
 * @brief 搜索号牌状态更新方法
 * 
 * 根据移动向量控制无人机移动，并检查是否发现有效号牌
 * 
 * @return 速度控制指令数组 [v_x, v_y, v_z, v_yaw]
 */
std::array<double,4> SearchLicenseState::on_update()
{
    if (std::get<0>(_move_vector)== 'y')
    {   
        _vel_cmd[1]=std::get<1>(_move_vector)*0.5;
        _vel_cmd[2]=0.0;
    }
    else if (std::get<0>(_move_vector)== 'z')
    {   
        _vel_cmd[1]=0.0;
        _vel_cmd[2]=std::get<1>(_move_vector)*0.5;
    }

    if (_license_processor->is_valid())
    {
        auto license_no=_license_processor->get_license_info_stable().license_no;
        ROS_INFO("[CL] 发现有效号牌 %s，准备确认。",license_no.c_str());
        static_cast<ConfirmLicenseMission&>(_mission).switch_node("confirm_license");
        return {0.0,0.0,0.0,0.0};
    }
    if (_license_processor->is_recognized())
    {
        if (std::get<0>(_move_vector)== 'y')
            // 调整Z轴,使号牌中心点处于检测框中
            _vel_cmd[2]=_license_processor->calculate_velocity('z');
        else if (std::get<0>(_move_vector)== 'z')
            // 调整Y轴,使号牌中心点处于检测框中
            _vel_cmd[1]=_license_processor->calculate_velocity('y');
    }
    return _vel_cmd;
}

/**
 * @brief 退出搜索号牌状态时调用
 * 
 * 目前为空实现
 */
void SearchLicenseState::on_exit()
{
}

/**
 * @brief ConfirmLicenseState构造函数
 * 
 * 初始化确认号牌状态节点，获取号牌处理器并设置超时时长
 * 
 * @param mission 所在任务引用
 */
ConfirmLicenseState::ConfirmLicenseState(MissionNode& mission):StateNode(mission)
{
    // 获取号牌处理器
    _license_processor=static_cast<ConfirmLicenseMission&>(_mission).get_license_processor();
    // 设置超时时长-该处为悬停等待时长
    _timeout_duration= ros::Duration(2.0);
}

/**
 * @brief 进入确认号牌状态时调用
 * 
 * 记录日志，清空已拒绝的号牌，并记录进入时间
 */
void ConfirmLicenseState::on_enter()
{
    ROS_INFO("[CL] ConfirmLicenseState::on_enter");
    // 清空已拒绝的号牌
    _license_processor->clear_rejected_plates();
    // 记录进入时间
    _entry_time= ros::Time::now();
}

/**
 * @brief 确认号牌状态更新方法
 * 
 * 检查号牌有效性，等待超时后比对号牌，成功则完成任务，失败则切换到重新定位状态
 * 
 * @return 速度控制指令数组 [v_x, v_y, v_z, v_yaw]
 */
std::array<double,4> ConfirmLicenseState::on_update()
{
    if (!_license_processor->is_valid())
    {
        static_cast<ConfirmLicenseMission&>(_mission).switch_node("search_license");
        return {0.0,0.0,0.0,0.0};
    }
    
    if (!is_timeout())
        return {0.0,0.0,0.0,0.0};

    if (_license_processor->compare_license())
    {
        ROS_INFO("[CL] 号牌比对成功");
        _mission.set_finished(true);
        return {0.0,0.0,0.0,0.0};
    }
    else
    {
        ROS_INFO("[CL] 号牌比对失败，切换到搜索状态");
        static_cast<ConfirmLicenseMission&>(_mission).switch_node("reposition_license");
        return {0.0,0.0,0.0,0.0};
    }
}

/**
 * @brief 退出确认号牌状态时调用
 * 
 * 目前为空实现
 */
void ConfirmLicenseState::on_exit()
{
}

/**
 * @brief RepositionLicenseState构造函数
 * 
 * 初始化重新定位状态节点，获取号牌处理器并设置超时时长
 * 
 * @param mission 所在任务引用
 */
RepositionLicenseState::RepositionLicenseState(MissionNode& mission):StateNode(mission)
{
    // 获取号牌处理器
    _license_processor=static_cast<ConfirmLicenseMission&>(_mission).get_license_processor();
    // 设置超时时长
    _timeout_duration= ros::Duration(1.7);
}

/**
 * @brief 进入重新定位状态时调用
 * 
 * 记录日志，计算新的移动向量，并更新号牌确认任务的移动方向
 */
void RepositionLicenseState::on_enter()
{
    ROS_INFO("[CL] RepositionLicenseState::on_enter");
    // 记录进入时间
    _entry_time= ros::Time::now();
    _new_move_vector=_license_processor->calculate_movement_from_plate_diff();
    if (std::get<0>(_new_move_vector)!= '\0')
    {
        static_cast<ConfirmLicenseMission&>(_mission).update_move_vector(_new_move_vector);
        ROS_INFO("[CL] 确定移动方向: %c轴，方向: %s",std::get<0>(_new_move_vector),std::get<1>(_new_move_vector)>=0.0?"正向":"反向");
    }
    else
        ROS_INFO("[CL] 保持之前移动方向");
}

/**
 * @brief 重新定位状态更新方法
 * 
 * 检查是否超时，超时则切换回搜索状态；否则根据新移动向量控制无人机移动
 * 
 * @return 速度控制指令数组 [v_x, v_y, v_z, v_yaw]
 */
std::array<double,4> RepositionLicenseState::on_update()
{
    // 检查是否超时
    if (is_timeout())
    {
        static_cast<ConfirmLicenseMission&>(_mission).switch_node("search_license");
        return {0.0,0.0,0.0,0.0};
    }
    // 根据新移动向量更新速度指令
    if (std::get<0>(_new_move_vector)== 'y')
    {   
        _vel_cmd[1]=std::get<1>(_new_move_vector)*0.5;
        _vel_cmd[2]=0.0;
    }
    else if (std::get<0>(_new_move_vector)== 'z')
    {   
        _vel_cmd[1]=0.0;
        _vel_cmd[2]=std::get<1>(_new_move_vector)*0.5;
    }
    return _vel_cmd;
}

/**
 * @brief 退出重新定位状态时调用
 * 
 * 目前为空实现
 */
void RepositionLicenseState::on_exit()
{
}