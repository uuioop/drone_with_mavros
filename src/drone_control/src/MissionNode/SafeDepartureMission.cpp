#include "MissionNode/SafeDepartureMission.h"
#include "StateNode/SafeDepartureNodes.h"

SafeDepartureMission::SafeDepartureMission(DroneControl& drone_control, ros::NodeHandle& nh, StatusMonitor& status_monitor, MavrosBridge& mavros_bridge)
    : MissionNode(drone_control, nh, status_monitor, mavros_bridge)
{
    _required_flight_mode ="OFFBOARD";
    _next_mission_name="idle";
    // 前方标记配置（初始对准）
    _front_tag_config.type = TagType::BOARD;
    _front_tag_config.topic_name = "board_detect/pose"; // 前方标记话题名称
    _front_tag_config.tag_name = "depart_front"; // 前方标记名称
    // 相机到机体的旋转矩阵(FLU坐标系)
    _front_tag_config.rotation_matrix << 0,0,1,
                                        -1,0,0,
                                        0,-1,0;
    _front_tag_config.alignment_axes = {'y','z','\0'}; // 对准轴：Y轴（左右）和Z轴（高度）
    _front_tag_config.target_timeout = 1.5; // 目标超时时间：1.5秒
    _front_tag_config.target_offset = Eigen::Vector3d(0.0, 0.0, 0.0); // 目标无偏移
    _front_tag_config.kp = 0.4; // PID比例系数
    _front_tag_config.max_speed = 0.4; // 最大移动速度：0.4米/秒
    
    // 下方平台标记配置（最终精准对准）
    _down_tag_config.topic_name = "/down_tracker_child/fiducial_transforms"; // 下方标记话题名称
    _down_tag_config.target_ids = {23,42}; // 下方标记目标ID
    _down_tag_config.tag_name = "depart_down"; // 下方标记名称
    // 最大检测距离，X和Y方向无限制，Z方向最大1.5米
    _down_tag_config.max_detection_distance = {
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        1.5
    };
    // 相机到机体的旋转矩阵(FLU坐标系)
    _down_tag_config.rotation_matrix << 0,-1,0,
                                       -1,0,0,
                                       0,0,-1;
    _down_tag_config.alignment_axes = {'x','y','\0'}; // 对准轴：X轴（前后）和Y轴（左右）
    _down_tag_config.aligned_tolerance = 0.15; // 对准容差：0.15米
    _down_tag_config.target_timeout = 1.0; // 目标超时时间：1.0秒
    _down_tag_config.target_offset = Eigen::Vector3d(0.0, 0.0, 0.0); // 目标偏移：无偏移
    _down_tag_config.kp = 0.4; // PID比例系数
    _down_tag_config.max_speed = 0.4; // 最大移动速度：0.4米/秒

    // 在配置完成后再创建处理器
    _front_tag_processor = std::make_shared<ArucoTagProcessor>(_nh, _front_tag_config);
    _down_tag_processor = std::make_shared<ArucoTagProcessor>(_nh, _down_tag_config);
    register_states();
}

SafeDepartureMission::~SafeDepartureMission()=default;

void SafeDepartureMission::register_states()
{
    // 注册搜索引导标记状态
    _state_machine.register_node("find_front_tag", std::make_shared<FindFrontTag>(*this));
    // 注册保持远离标记状态
    _state_machine.register_node("keep_away_from_tag", std::make_shared<KeepAwayFromTag>(*this));
    // 注册盲飞状态
    _state_machine.register_node("blind_state", std::make_shared<BlindState>(*this));
    // 注册搜索信号状态
    _state_machine.register_node("search_signal", std::make_shared<SearchSignalState>(*this));
}

void SafeDepartureMission::on_enter()
{
    MissionNode::on_enter();
    // 设置初始状态为搜索引导标记
    _state_machine.set_entry("find_front_tag");
    // 启动前置摄像头标记处理器
    _front_tag_processor->set_start(true);
    // 启动下视摄像头标记处理器
    _down_tag_processor->set_start(true);
}

std::array<double,4> SafeDepartureMission::on_update()
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

void SafeDepartureMission::on_exit()
{
    // 关闭前置摄像头标记处理器
    _front_tag_processor->set_start(false);
    // 关闭下视摄像头标记处理器
    _down_tag_processor->set_start(false);
}