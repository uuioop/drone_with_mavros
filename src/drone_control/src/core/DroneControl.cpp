/**
 * @file DroneControl.cpp
 * @brief DroneControl类的实现文件，提供无人机的整体控制功能
 */
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <cmath>  // 用于数学函数cos, sin, M_PI
#include <iostream>

#include "core/DroneControl.h"
#include "MissionNode/GPSNavMission.h"
#include "MissionNode/ConfirmLicenseMission.h"
#include "MissionNode/PrecisionLandMission.h"
#include "MissionNode/IdleMission.h"
#include "MissionNode/SafeDepartureMission.h"

/**
 * @brief DroneControl类构造函数
 * @note 使用初始化列表直接进行初始化，而不是"先默认构造再赋值"
 * @note 成员变量中的引用类型必须在成员初始化列表中进行初始化
 */
DroneControl::DroneControl()
: _nh(),
_status_monitor(_nh),
_mavros_bridge(_nh, _status_monitor)
{
	register_missions();
	_mission_machine.set_entry("idle");
}

/**
 * @brief DroneControl类析构函数
 * @note 使用默认实现
 */
DroneControl::~DroneControl()=default;

/**
 * @brief 注册所有可用的任务
 * @note 将各任务实例化并添加到任务池中
 */
void DroneControl::register_missions()
{
	// 注册所有任务
	_mission_machine.register_node("idle", std::make_shared<IdleMission>(*this, _nh, _status_monitor, _mavros_bridge));
	_mission_machine.register_node("mission_nav", std::make_shared<GPSNavMission>(*this, _nh, _status_monitor, _mavros_bridge));
	_mission_machine.register_node("confirm_license", std::make_shared<ConfirmLicenseMission>(*this, _nh, _status_monitor, _mavros_bridge));
	_mission_machine.register_node("precision_land", std::make_shared<PrecisionLandMission>(*this, _nh, _status_monitor, _mavros_bridge));
	_mission_machine.register_node("safe_departure", std::make_shared<SafeDepartureMission>(*this, _nh, _status_monitor, _mavros_bridge));
}

/**
 * @brief 更新控制器状态，主控制循环函数
 * @note 检查当前控制器是否完成任务，必要时切换控制器
 * @note 调用当前活动控制器的on_update方法
 */
void DroneControl::on_update()
{
	_mission_machine.on_update();
}

/**
 * @brief 切换当前任务
 * @param mission_name 目标任务名称
 */
void DroneControl::switch_mission(const std::string& mission_name)
{
    _mission_machine.switch_node(mission_name);
}