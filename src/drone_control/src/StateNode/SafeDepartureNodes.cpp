#include "MissionNode/SafeDepartureMission.h"
#include "StateNode/SafeDepartureNodes.h"

/**
 * @brief 构造函数
 * @param mission 所在任务引用
 */
FindFrontTag::FindFrontTag(MissionNode& mission):StateNode(mission)
{
    _front_tag_processor=static_cast<SafeDepartureMission&>(_mission).get_front_tag_processor();
    _down_tag_processor=static_cast<SafeDepartureMission&>(_mission).get_down_tag_processor();
}

void FindFrontTag::on_enter()
{
    ROS_INFO("[SD] 进入：寻找引导标记状态");
}

std::array<double,4> FindFrontTag::on_update()
{
    if(_front_tag_processor->is_valid())
    {
        ROS_INFO("[SD] 找到引导标记，开始远离...");
        static_cast<SafeDepartureMission&>(_mission).switch_node("keep_away_from_tag");
        return {0.0,0.0,0.0,0.0};
    }

    if(_down_tag_processor->is_valid())
    {
        try {
            _alignment_errors=_down_tag_processor->get_alignment_errors();
        } catch (const std::runtime_error& e) {
            ROS_ERROR("[SD] 寻找引导标记状态, 获取误差向量失败: %s", e.what());
            return {0.0, 0.0, 0.0, 0.0};
        }
        // 若到达一定高度还未找到引导标记，切换到盲飞状态
        if(std::abs(_alignment_errors[2])>1.5)
        {
            ROS_INFO("11");
            static_cast<SafeDepartureMission&>(_mission).switch_node("blind_state");
            return {0.0,0.0,0.0,0.0};
        }
        _vel_cmd=_down_tag_processor->calculate_velocity_command(false);
        // 反向Z轴速度远离下方平台标记
        _vel_cmd[2]=-_vel_cmd[2];
        return _vel_cmd;
    }
    return {0.0,0.0,0.4,0.0};
}

void FindFrontTag::on_exit()
{
    // 退出发现引导标记状态
}

KeepAwayFromTag::KeepAwayFromTag(MissionNode& mission):StateNode(mission)
{
    _front_tag_processor=static_cast<SafeDepartureMission&>(_mission).get_front_tag_processor();
}

void KeepAwayFromTag::on_enter()
{
    ROS_INFO("[SD] 进入：远离引导标记状态");
    _entry_time= ros::Time::now();
}

std::array<double,4> KeepAwayFromTag::on_update()
{
    if(_front_tag_processor->is_valid())
    {
        // 获取引导标记的对准误差向量
        try {
            _alignment_errors=_front_tag_processor->get_alignment_errors();
        } catch (const std::runtime_error& e) {
            ROS_ERROR("[SD] 远离引导标记状态, 获取误差向量失败: %s", e.what());
            return {0.0, 0.0, 0.0, 0.0};
        }
    }
    if(!_front_tag_processor->is_valid() || std::abs(_alignment_errors[0])>1.8)
    {
        ROS_INFO("[SD] 引导标记已消失或远离目标，启动GPS返航");
        _mission.set_finished(true);
        return {0.0,0.0,0.0,0.0};
    }
    _vel_cmd=_front_tag_processor->calculate_velocity_command(true);
    // 反向X轴速度远离引导标记
    _vel_cmd[0]=-_vel_cmd[0];
    return _vel_cmd;
}

void KeepAwayFromTag::on_exit()
{
    // 退出远离引导标记状态
}

BlindState::BlindState(MissionNode& mission):StateNode(mission)
{
    _timeout_duration=ros::Duration(5.0);
    _front_tag_processor=static_cast<SafeDepartureMission&>(_mission).get_front_tag_processor();
}

void BlindState::on_enter()
{
    ROS_INFO("[SD] 进入：盲飞状态");
    _entry_time= ros::Time::now();
}

std::array<double,4> BlindState::on_update()
{
    if(is_timeout())
    {
        ROS_INFO("[SD] 盲飞时间结束，开始GPS导航");
        _mission.set_finished(true);
        return {0.0,0.0,0.0,0.0};
    }
    if(_front_tag_processor->is_valid())
    {
        ROS_INFO("[SD] 找到引导标记，切换到远离引导标记状态");
        static_cast<SafeDepartureMission&>(_mission).switch_node("keep_away_from_tag");
        return {0.0,0.0,0.0,0.0};
    }
    return {-0.3,0.0,0.0,0.0};
}

void BlindState::on_exit()
{
    // 退出盲飞状态
}