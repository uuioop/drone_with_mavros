#include "util/LicenseDatabase.h"

/**
 * @brief LicenseDatabase类的实现文件
 */

/**
 * @brief 初始化数据库表结构
 * @return 是否初始化成功
 */
bool LicenseDatabase::_initDatabase() {
    try {
        // 打开数据库连接
        int result = sqlite3_open(_db_path.c_str(), &_connection);
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 打开数据库失败: " << sqlite3_errmsg(_connection) << std::endl;
            sqlite3_close(_connection);
            _connection = nullptr;
            return false;
        }

        const char* create_table_sql = "CREATE TABLE IF NOT EXISTS license_plates "
                                     "(plate_number TEXT PRIMARY KEY NOT NULL, "
                                     "latitude REAL DEFAULT 0.0, "
                                     "longitude REAL DEFAULT 0.0, "
                                     "altitude REAL DEFAULT 0.0, "
                                     "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP)";

        char* error_message = nullptr;
        result = sqlite3_exec(_connection, create_table_sql, nullptr, nullptr, &error_message);
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 创建表失败: " << error_message << std::endl;
            sqlite3_free(error_message);
            sqlite3_close(_connection);
            _connection = nullptr;
            return false;
        }

        std::cout << "[数据库] 数据库初始化完成: " << _db_path << std::endl;
        _is_initialized = true;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[数据库] 初始化失败: " << e.what() << std::endl;
        if (_connection) {
            sqlite3_close(_connection);
            _connection = nullptr;
        }
        return false;
    }
}

/**
 * @brief 构造函数
 * @param db_path 数据库文件路径
 */
LicenseDatabase::LicenseDatabase(const std::string& db_path) : 
    _db_path(db_path), 
    _connection(nullptr), 
    _is_initialized(false) {
    _initDatabase();
}

/**
 * @brief 析构函数
 */
LicenseDatabase::~LicenseDatabase() {
    close();
}

/**
 * @brief 添加或更新号牌到数据库
 * @param plate_number 号牌号码
 * @param latitude 纬度
 * @param longitude 经度
 * @param altitude 高度
 * @return 是否添加/更新成功
 */
bool LicenseDatabase::addLicensePlate(const std::string& plate_number, 
                                     double latitude, 
                                     double longitude, 
                                     double altitude) {
    if (!_is_initialized || !_connection) {
        std::cerr << "[数据库] 数据库未初始化" << std::endl;
        return false;
    }

    try {
        const char* insert_sql = "INSERT OR REPLACE INTO license_plates "
                               "(plate_number, latitude, longitude, altitude, timestamp) "
                               "VALUES (?, ?, ?, ?, CURRENT_TIMESTAMP);";

        sqlite3_stmt* stmt = nullptr;
        int result = sqlite3_prepare_v2(_connection, insert_sql, -1, &stmt, nullptr);
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 准备SQL语句失败: " << sqlite3_errmsg(_connection) << std::endl;
            return false;
        }

        sqlite3_bind_text(stmt, 1, plate_number.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_double(stmt, 2, latitude);
        sqlite3_bind_double(stmt, 3, longitude);
        sqlite3_bind_double(stmt, 4, altitude);

        result = sqlite3_step(stmt);
        if (result != SQLITE_DONE) {
            std::cerr << "[数据库] 执行SQL语句失败: " << sqlite3_errmsg(_connection) << std::endl;
            sqlite3_finalize(stmt);
            return false;
        }

        sqlite3_finalize(stmt);
        sqlite3_exec(_connection, "COMMIT", nullptr, nullptr, nullptr);
        std::cout << "[数据库] 添加/更新号牌成功: " << plate_number << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[数据库] 添加/更新号牌失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 从数据库移除号牌
 * @param plate_number 号牌号码
 * @return 是否移除成功
 */
bool LicenseDatabase::removeLicensePlate(const std::string& plate_number) {
    if (!_is_initialized || !_connection) {
        std::cerr << "[数据库] 数据库未初始化" << std::endl;
        return false;
    }

    try {
        const char* delete_sql = "DELETE FROM license_plates WHERE plate_number = ?;";
        sqlite3_stmt* stmt = nullptr;
        int result = sqlite3_prepare_v2(_connection, delete_sql, -1, &stmt, nullptr);
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 准备SQL语句失败: " << sqlite3_errmsg(_connection) << std::endl;
            return false;
        }

        sqlite3_bind_text(stmt, 1, plate_number.c_str(), -1, SQLITE_TRANSIENT);
        result = sqlite3_step(stmt);
        sqlite3_finalize(stmt);

        sqlite3_exec(_connection, "COMMIT", nullptr, nullptr, nullptr);
        std::cout << "[数据库] 移除号牌成功: " << plate_number << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[数据库] 移除号牌失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 获取所有号牌
 * @return 号牌列表
 */
std::vector<LicensePlate> LicenseDatabase::getAllLicensePlates() {
    std::vector<LicensePlate> license_plates;
    
    if (!_is_initialized || !_connection) {
        std::cerr << "[数据库] 数据库未初始化" << std::endl;
        return license_plates;
    }

    try {
        const char* select_sql = "SELECT * FROM license_plates;";
        sqlite3_stmt* stmt = nullptr;
        int result = sqlite3_prepare_v2(_connection, select_sql, -1, &stmt, nullptr);
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 准备SQL语句失败: " << sqlite3_errmsg(_connection) << std::endl;
            return license_plates;
        }

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            LicensePlate plate;
            plate.plate_number = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            plate.latitude = sqlite3_column_double(stmt, 1);
            plate.longitude = sqlite3_column_double(stmt, 2);
            plate.altitude = sqlite3_column_double(stmt, 3);
            plate.timestamp = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
            license_plates.push_back(plate);
        }

        sqlite3_finalize(stmt);

    } catch (const std::exception& e) {
        std::cerr << "[数据库] 获取号牌列表失败: " << e.what() << std::endl;
    }

    return license_plates;
}

/**
 * @brief 检查号牌是否存在
 * @param plate_number 号牌号码
 * @return 是否存在
 */
bool LicenseDatabase::isLicensePlateExists(const std::string& plate_number) {
    if (!_is_initialized || !_connection) {
        std::cerr << "[数据库] 数据库未初始化" << std::endl;
        return false;
    }

    try {
        const char* select_sql = "SELECT COUNT(*) FROM license_plates WHERE plate_number = ?;";
        sqlite3_stmt* stmt = nullptr;
        int result = sqlite3_prepare_v2(_connection, select_sql, -1, &stmt, nullptr);
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 准备SQL语句失败: " << sqlite3_errmsg(_connection) << std::endl;
            return false;
        }

        sqlite3_bind_text(stmt, 1, plate_number.c_str(), -1, SQLITE_TRANSIENT);
        result = sqlite3_step(stmt);
        
        int count = 0;
        if (result == SQLITE_ROW) {
            count = sqlite3_column_int(stmt, 0);
        }

        sqlite3_finalize(stmt);
        return count > 0;

    } catch (const std::exception& e) {
        std::cerr << "[数据库] 检查号牌存在性失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 清空数据库
 * @return 是否清空成功
 */
bool LicenseDatabase::clearDatabase() {
    if (!_is_initialized || !_connection) {
        std::cerr << "[数据库] 数据库未初始化" << std::endl;
        return false;
    }

    try {
        const char* delete_sql = "DELETE FROM license_plates;";
        char* error_message = nullptr;
        int result = sqlite3_exec(_connection, delete_sql, nullptr, nullptr, &error_message);
        
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 清空数据库失败: " << error_message << std::endl;
            sqlite3_free(error_message);
            return false;
        }

        sqlite3_exec(_connection, "COMMIT", nullptr, nullptr, nullptr);
        std::cout << "[数据库] 数据库已清空" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[数据库] 清空数据库失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 获取数据库统计信息
 * @return 统计信息
 */
std::map<std::string, int> LicenseDatabase::getDatabaseStats() {
    std::map<std::string, int> stats;
    stats["total_plates"] = 0;
    
    if (!_is_initialized || !_connection) {
        std::cerr << "[数据库] 数据库未初始化" << std::endl;
        return stats;
    }

    try {
        const char* select_sql = "SELECT COUNT(*) FROM license_plates;";
        sqlite3_stmt* stmt = nullptr;
        int result = sqlite3_prepare_v2(_connection, select_sql, -1, &stmt, nullptr);
        if (result != SQLITE_OK) {
            std::cerr << "[数据库] 准备SQL语句失败: " << sqlite3_errmsg(_connection) << std::endl;
            return stats;
        }

        result = sqlite3_step(stmt);
        if (result == SQLITE_ROW) {
            stats["total_plates"] = sqlite3_column_int(stmt, 0);
        }

        sqlite3_finalize(stmt);

    } catch (const std::exception& e) {
        std::cerr << "[数据库] 获取统计信息失败: " << e.what() << std::endl;
    }

    return stats;
}

/**
 * @brief 关闭数据库连接
 */
void LicenseDatabase::close() {
    if (_connection) {
        sqlite3_close(_connection);
        _connection = nullptr;
        _is_initialized = false;
        std::cout << "[数据库] 数据库连接已关闭" << std::endl;
    }
}