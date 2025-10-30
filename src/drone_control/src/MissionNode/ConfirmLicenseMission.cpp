/**
 * @file ConfirmLicenseMission.cpp
 * @brief 号牌确认任务实现
 * 
 * 该任务负责在视觉导航中确认无人机当前所在平台的号牌。
 * 它使用状态机管理不同的确认流程，包括搜索号牌、确认号牌和重新定位号牌。
 */
#include "MissionNode/ConfirmLicenseMission.h"
#include "StateNode/ConfirmLicenseNodes.h"

/**
 * @brief 构造函数实现
 * 初始化号牌处理器，注册状态机中的状态，并设置初始状态为搜索号牌
 * @param nh ROS节点句柄
 * @param status_monitor 状态监控器引用
 * @param mavros_bridge MAVROS桥接器引用
 */
ConfirmLicenseMission::ConfirmLicenseMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge):
    MissionNode(drone_control, nh, status_monitor, mavros_bridge)
{// 保持默认参数_license_config
    _license_processor=std::make_shared<LicenseProcessor>(_nh,_license_config);
    _required_flight_mode ="OFFBOARD";
    _next_mission_name="precision_land";
    // 注册状态机状态
    register_states();
}

/**
 * @brief 析构函数实现
 * 使用默认析构函数，自动管理资源释放
 */
ConfirmLicenseMission::~ConfirmLicenseMission()=default;

/**
 * @brief 注册状态机中的各个状态
 * 将搜索号牌、确认号牌和重新定位号牌三个状态注册到状态机中
 */
void ConfirmLicenseMission::register_states()
{
    // 注册状态机状态
    _state_machine.register_node("search_license",std::make_shared<SearchLicenseState>(*this));
    _state_machine.register_node("confirm_license",std::make_shared<ConfirmLicenseState>(*this));
    _state_machine.register_node("reposition_license",std::make_shared<RepositionLicenseState>(*this));
}

/**
 * @brief 进入任务时的回调实现
 * 目前为空实现，可根据需要添加初始化逻辑
 */
void ConfirmLicenseMission::on_enter()
{
    MissionNode::on_enter();
    // 设置初始状态为搜索号牌
    _state_machine.set_entry("search_license");
    _license_processor->set_start(true);
}

/**
 * @brief 更新任务状态时的回调实现
 * 获取当前状态的控制指令并通过MAVROS桥接器发送给无人机
 */
std::array<double,4> ConfirmLicenseMission::on_update()
{
    _mavros_bridge.set_velocity_body(_state_machine.on_update());
    // 等待下一个循环周期
    _rate.sleep();
    return MissionNode::on_update();
}

/**
 * @brief 退出任务时的回调实现
 * 目前为空实现，可根据需要添加清理逻辑
 */
void ConfirmLicenseMission::on_exit()
{
    _license_processor->set_start(false);
}

