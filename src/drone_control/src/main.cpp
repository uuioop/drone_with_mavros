/**
 * @file main.cpp
 * @brief 无人机控制节点主入口文件
 * @details 初始化ROS节点，创建DroneControl实例并运行主控制循环
 */
#include <clocale>

#include "core/DroneControl.h"

/**
 * @brief 主函数，无人机控制节点的入口点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码，成功执行返回0
 * @note 使用AsyncSpinner处理ROS回调，确保多订阅者正常工作
 * @note 运行持续的控制循环，直到ROS节点关闭
 */
int main(int argc, char **argv)
{
    // 设置区域设置，支持UTF-8编码
    setlocale(LC_ALL, "");
    
    // 初始化ROS节点，节点名称为"drone_control_node"
    ros::init(argc, argv, "drone_control_node");
    
    /**
     * @brief 创建异步旋转器
     * @note 使用2个线程处理回调，确保服务和话题回调能同时处理
     * @note 多线程可以避免服务回调阻塞话题回调的执行
     */
    ros::AsyncSpinner spinner(2);
    spinner.start(); // 启动后台回调线程

    // 创建DroneControl实例
    DroneControl drone_control;
    
    /**
     * @brief 主控制循环
     * @note 持续调用on_update()方法，保持控制逻辑运行
     * @note 当ROS节点关闭时退出循环
     */
    while (ros::ok())
    {
        drone_control.on_update();
    }
    
    return 0;
}