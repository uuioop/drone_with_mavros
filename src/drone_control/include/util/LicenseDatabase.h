#pragma once
#include <string>
#include <vector>
#include <map>
#include <sqlite3.h>
#include <iostream>
#include <ctime>

/**
 * @file LicenseDatabase.h
 * @brief 单一号牌表实现，用于号牌管理，包含地理位置信息
 */

/**
 * @struct LicensePlate
 * @brief 号牌信息结构体
 */
struct LicensePlate {
    std::string plate_number; ///< 号牌号码
    double latitude;          ///< 纬度
    double longitude;         ///< 经度
    double altitude;          ///< 高度
    std::string timestamp;    ///< 时间戳
};

/**
 * @class LicenseDatabase
 * @brief 号牌SQLite数据库类
 * 管理号牌信息的数据库操作，包括添加、更新、删除和查询号牌记录
 */
class LicenseDatabase {
private:
    std::string _db_path;     ///< 数据库文件路径
    sqlite3* _connection;     ///< SQLite连接指针
    bool _is_initialized;     ///< 数据库是否初始化标志

    /**
     * @brief 初始化数据库表结构
     * @return 是否初始化成功
     */
    bool _initDatabase();

public:
    /**
     * @brief 构造函数
     * @param db_path 数据库文件路径，默认为"license_plates.db"
     */
    LicenseDatabase(const std::string& db_path = "license_plates.db");

    /**
     * @brief 析构函数
     * 确保关闭数据库连接
     */
    ~LicenseDatabase();

    /**
     * @brief 添加或更新号牌到数据库
     * @param plate_number 号牌号码
     * @param latitude 纬度，默认为0.0
     * @param longitude 经度，默认为0.0
     * @param altitude 高度，默认为0.0
     * @return 是否添加/更新成功
     */
    bool addLicensePlate(const std::string& plate_number, 
                        double latitude = 0.0, 
                        double longitude = 0.0, 
                        double altitude = 0.0);

    /**
     * @brief 从数据库移除号牌
     * @param plate_number 号牌号码
     * @return 是否移除成功
     */
    bool removeLicensePlate(const std::string& plate_number);

    /**
     * @brief 获取所有号牌
     * @return 号牌列表
     */
    std::vector<LicensePlate> getAllLicensePlates();

    /**
     * @brief 检查号牌是否存在
     * @param plate_number 号牌号码
     * @return 是否存在
     */
    bool isLicensePlateExists(const std::string& plate_number);

    /**
     * @brief 清空数据库
     * @return 是否清空成功
     */
    bool clearDatabase();

    /**
     * @brief 获取数据库统计信息
     * @return 统计信息，包含总号牌数
     */
    std::map<std::string, int> getDatabaseStats();

    /**
     * @brief 关闭数据库连接
     */
    void close();

    /**
     * @brief 检查数据库是否已初始化
     * @return 是否已初始化
     */
    bool isInitialized() const {
        return _is_initialized;
    }
};