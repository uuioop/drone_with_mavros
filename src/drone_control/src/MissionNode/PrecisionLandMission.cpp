/**
 * @file PrecisionLandMission.cpp
 * @brief 精确着陆任务节点实现文件
 * 
 * 该文件实现了无人机的精确着陆任务节点，包括前置和下视摄像头标记配置、状态机管理、
 * 状态机中的状态实现等功能，用于无人机的精确着陆任务。
 */
#include <limits>
#include <ros/ros.h>
#include "MissionNode/PrecisionLandMission.h"
#include "StateNode/PrecisionLandNodes.h"

/**
 * @brief 构造函数实现
 * 初始化前置和下视摄像头的标签配置，创建标签处理器，注册状态机中的状态
 * @param nh ROS节点句柄
 * @param status_monitor 状态监控器引用
 * @param mavros_bridge MAVROS桥接器引用
 */
PrecisionLandMission::PrecisionLandMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge):
MissionNode(drone_control, nh, status_monitor, mavros_bridge)
{
    _required_flight_mode ="OFFBOARD";
    _next_mission_name="safe_departure";
    // 前方标记配置（初始对准）
    _front_tag_config.type = TagType::BOARD;
    _front_tag_config.topic_name = "board_detect/pose"; // 前方标记话题名称
    _front_tag_config.tag_name = "land_front"; // 前方标记名称
    // 相机到机体的旋转矩阵(FLU坐标系)
    _front_tag_config.rotation_matrix << 0,0,1,
                                        -1,0,0,
                                        0,-1,0;
    _front_tag_config.alignment_axes = {'y','z','\0'}; // 对准轴：Y轴（左右）和Z轴（高度）
    _front_tag_config.aligned_tolerance = 0.2; // 对准容差：0.2米
    _front_tag_config.target_timeout = 0.5; // 目标超时时间：0.5秒
    _front_tag_config.target_offset = Eigen::Vector3d(0.0, 0.0, 0.0); // 目标偏移：前方1.4米
    _front_tag_config.kp = 0.4; // PID比例系数
    _front_tag_config.max_speed = 0.4; // 最大移动速度：0.4米/秒
    
    // 下方平台标记配置（最终精准对准）
    _down_tag_config.type = TagType::SINGLE;
    _down_tag_config.topic_name = "/down_aruco_tracker/fiducial_transforms"; // 下方标记话题名称
    _down_tag_config.target_ids = {23,42}; // 下方标记目标ID
    _down_tag_config.tag_name = "land_down"; // 下方标记名称
    // 最大检测距离，X和Y方向无限制，Z方向最大2.7米
    _down_tag_config.max_detection_distance = {
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        2.7
    };
    // 相机到机体的旋转矩阵(FLU坐标系)
    _down_tag_config.rotation_matrix << 0,-1,0,
                                       -1,0,0,
                                       0,0,-1;
    _down_tag_config.alignment_axes = {'x','y','\0'}; // 对准轴：X轴（前后）和Y轴（左右）
    _down_tag_config.aligned_tolerance = 0.2; // 对准容差：0.2米
    _down_tag_config.target_timeout = 0.5; // 目标超时时间：0.5秒
    _down_tag_config.target_offset = Eigen::Vector3d(0.0, 0.0, 0.0); // 目标偏移：无偏移
    _down_tag_config.kp = 0.4; // PID比例系数
    _down_tag_config.max_speed = 0.4; // 最大移动速度：0.4米/秒

    // 在配置完成后再创建处理器
    _front_tag_processor = std::make_shared<ArucoTagProcessor>(_nh, _front_tag_config);
    _down_tag_processor = std::make_shared<ArucoTagProcessor>(_nh, _down_tag_config);
    // 注册状态机状态
    register_states();

}

/**
 * @brief 析构函数实现
 * 使用默认析构函数，自动管理资源释放
 */
PrecisionLandMission::~PrecisionLandMission()=default;

/**
 * @brief 注册状态机中的各个状态
 * 将精确着陆过程中的六个状态依次注册到状态机中
 */
void PrecisionLandMission::register_states()
{
    // 注册状态机状态
    _state_machine.register_node("search_tag", std::make_shared<SearchTagState>(*this));
    _state_machine.register_node("align_on_guide_tag", std::make_shared<AlignOnGuideTagState>(*this));
    _state_machine.register_node("approach_guide_tag", std::make_shared<ApproachGuideTagState>(*this));
    _state_machine.register_node("align_on_platform_tag", std::make_shared<AlignOnPlatformTagState>(*this));
    _state_machine.register_node("descend", std::make_shared<DescendState>(*this));
    _state_machine.register_node("land", std::make_shared<LandState>(*this));
}

/**
 * @brief 进入精确着陆任务时的回调实现
 * 启动标记处理器，开始处理标记检测数据
 */
void PrecisionLandMission::on_enter()
{
    MissionNode::on_enter();
    // 设置初始状态为搜索标记
    _state_machine.set_entry("search_tag");
    // 启动前置摄像头标记处理器
    _front_tag_processor->set_start(true);
    // 启动下视摄像头标记处理器
    _down_tag_processor->set_start(true);
}
/**
 * @brief 更新精确着陆任务状态时的回调实现
 * 更新标签处理器的有效性状态，并获取当前状态的控制指令通过MAVROS桥接器发送给无人机
 */
std::array<double,4> PrecisionLandMission::on_update()
{
    // 更新前置摄像头标签处理器的有效性状态
    _front_tag_processor->update_and_log_validity();
    // 更新下视摄像头标签处理器的有效性状态
    _down_tag_processor->update_and_log_validity();
    // 获取当前状态的控制指令并发送
    _mavros_bridge.set_velocity_body(_state_machine.on_update());
    // 等待下一个循环周期
    _rate.sleep();
    return MissionNode::on_update();
}

/**
 * @brief 退出精确着陆任务时的回调实现
 * 目前为空实现，可根据需要添加清理逻辑
 */
void PrecisionLandMission::on_exit()
{
    // 关闭前置摄像头标记处理器
    _front_tag_processor->set_start(false);
    // 关闭下视摄像头标记处理器
    _down_tag_processor->set_start(false);
}