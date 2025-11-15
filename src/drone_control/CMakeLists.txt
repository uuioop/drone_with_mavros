cmake_minimum_required(VERSION 3.0.2)
project(drone_control)

## 编译为C++11，在ROS Kinetic及更新版本中支持
# add_compile_options(-std=c++11)
# 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)

# 强制要求使用此标准，如果编译器不支持则会报错
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找catkin宏和库
## 如果使用COMPONENTS列表，如find_package(catkin REQUIRED COMPONENTS xyz)
## 也会查找其他catkin包
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  my_interfaces
  roscpp
  std_msgs
  sensor_msgs
)



## 系统依赖项使用CMake的约定查找
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
# 查找sqlite3库
find_package(SQLite3 REQUIRED)

################################################
## 声明ROS消息、服务和动作 ##
################################################

## 要在包内声明和构建消息、服务或动作，请遵循以下步骤：
## * 让MSG_DEP_SET成为您在消息/服务/动作中使用的消息类型的包集（例如std_msgs、actionlib_msgs等）。
## * 在package.xml文件中：
##   * 添加一个build_depend标签为"message_generation"
##   * 为MSG_DEP_SET中的每个包添加build_depend和exec_depend标签
##   * 如果MSG_DEP_SET不为空，以下依赖项已经被引入
##     但为了确定性仍然可以声明：
##     * 添加一个exec_depend标签为"message_runtime"
## * 在这个文件（CMakeLists.txt）中：
##   * 添加"message_generation"和MSG_DEP_SET中的每个包到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 添加"message_runtime"和MSG_DEP_SET中的每个包到
##     catkin_package(CATKIN_DEPENDS ...)
##   * 根据需要取消注释下面的add_*_files部分
##     并列出每个要处理的.msg/.srv/.action文件
##   * 取消注释下面的generate_messages条目
##   * 将MSG_DEP_SET中的每个包添加到generate_messages(DEPENDENCIES ...)

## 在'msg'文件夹中生成消息
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## 在'srv'文件夹中生成服务
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## 在'action'文件夹中生成动作
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 生成带有列出的依赖项的已添加消息和服务
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   mavros_msgs#   std_msgs
# )

################################################
## 声明ROS动态重配置参数 ##
################################################

## 要在此包中声明和构建动态重配置参数，请遵循以下步骤：
## * 在package.xml文件中：
##   * 添加build_depend和exec_depend标签为"dynamic_reconfigure"
## * 在这个文件（CMakeLists.txt）中：
##   * 添加"dynamic_reconfigure"到
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * 取消注释下面的"generate_dynamic_reconfigure_options"部分
##     并列出每个要处理的.cfg文件

## 在'cfg'文件夹中生成动态重配置参数
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin特定配置 ##
###################################
## catkin_package宏为您的包生成cmake配置文件
## 声明传递给依赖项目的内容
## INCLUDE_DIRS：如果包包含头文件，取消注释这一行
## LIBRARIES：您在此项目中创建的依赖项目也需要的库
## CATKIN_DEPENDS：依赖项目也需要的catkin包
## DEPENDS：依赖项目也需要的此项目的系统依赖项
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES drone_control
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs sensor_msgs my_interfaces
  DEPENDS Eigen3 SQLite3
)

###########
## 构建 ##
###########

## 指定头文件的其他位置
## 您的包位置应在其他位置之前列出
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
)

## 声明C++库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/drone_control.cpp
# )

## 添加库的cmake目标依赖项
## 例如，代码可能需要在库之前从消息生成或动态重配置生成
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 声明C++可执行文件
## 使用catkin_make，所有包都在单个CMake上下文中构建
## 推荐的前缀确保跨包的目标名称不会冲突
# add_executable(${PROJECT_NAME}_node src/drone_control_node.cpp)
# 从src目录递归查找所有.cpp文件
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/core/*.cpp" "src/StateNode/*.cpp" "src/Tag/*.cpp" "src/MissionNode/*.cpp")
add_executable(${PROJECT_NAME}_node ${SOURCES})

## 重命名不带前缀的C++可执行文件
## 上面推荐的前缀会导致目标名称过长，以下将目标重命名为较短版本以便用户使用
## 例如，"rosrun someones_pkg node"而不是"rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## 添加可执行文件的cmake目标依赖项
## 与上面的库相同
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定要链接库或可执行目标的库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${SQLite3_LIBRARIES}
)

#############
## 安装 ##
#############

# 所有安装目标都应使用catkin DESTINATION变量
# 参见http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记要安装的可执行脚本（Python等）
## 与setup.py相比，您可以选择目标位置
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记要安装的可执行文件
## 参见http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记要安装的库
## 参见http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## 标记要安装的cpp头文件
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 标记其他要安装的文件（例如launch和bag文件等）
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## 测试 ##
#############

## 添加基于gtest的cpp测试目标并链接库
# catkin_add_gtest(${PROJECT_NAME}-test test/test_drone_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## 添加要由python nosetests运行的文件夹
# catkin_add_nosetests(test)
