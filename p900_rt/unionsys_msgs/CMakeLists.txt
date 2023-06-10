cmake_minimum_required(VERSION 2.8)
project(unionsys_msgs)

#-------------------------------------- 编译器设置 --------------------------------------
# 版本号
set(LDROBOT_PROJECT_MAJOR_VERSION 2019)
set(LDROBOT_PROJECT_MINOR_VERSION 06)
set(LDROBOT_PROJECT_PATCH_VERSION 06)
set(LDROBOT_PROJECT_VERSION ${LDROBOT_PROJECT_MAJOR_VERSION}.${LDROBOT_PROJECT_MINOR_VERSION}.${LDROBOT_PROJECT_PATCH_VERSION})
#编译参数说明：
# -O 优化等级（0,不做优化 1,默认优化 2，继续优化 数字越大，优化程度越高）
# -Wall 警告信息全开
# -g 生成调试信息 （0: 少量调试信息， 1：默认， 2：更多的调试信息）
# -ggdb 把gdb格式的调试信息输出
# -Dmacro=define 定义宏
# -Umacro 取消定义宏
# -static 使用静态编译，禁止使用动态共享库
# -shared -fPIC 生成动态共享库
# -std=XXX： 使用的c++标准
set(CMAKE_BUILD_TYPE Release )
#set(CMAKE_BUILD_TYPE Debug )
if( CMAKE_BUILD_TYPE STREQUAL "Release")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -fPIC")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -fPIC")
endif()
message("*** ${PROJECT_NAME}: Build type:" ${CMAKE_BUILD_TYPE} ${CMAKE_CXX_FLAGS} "***")
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakes)
###
if(WIN32 AND NOT CYGWIN)
    set(HOME $ENV{PROFILE})
else()
    set(HOME $ENV{HOME})
endif()
set(GENERATED_HEADER_FILES_DIR ${PROJECT_BINARY_DIR}/generated_header_files/include)
string(TOUPPER "${PROJECT_NAME}" LDROBOT_UPPER_PROJECT_NAME)
string(TOLOWER "${PROJECT_NAME}" LDROBOT_LOWER_PROJECT_NAME)

#-------------------------------------- 搜索软件包 --------------------------------------
find_package(catkin REQUIRED COMPONENTS
roscpp
actionlib_msgs
std_msgs
std_srvs
geometry_msgs
sensor_msgs
message_generation
visualization_msgs
tf
)

#-------------------------------------- -添加项目- --------------------------------------

add_message_files(
DIRECTORY msg
FILES TrackRectangle.msg
      Gimbal_PidVel_cmd.msg)

#add_service_files(
#DIRECTORY srv
#FILES NaviStatusSwitch.srv
#      nlp.srv
#      RequestDockState.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

#-------------------------------------- 添加头文件 --------------------------------------

#--------------------------------------  生成项目 --------------------------------------


#--------------------------------------  Install --------------------------------------
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
DESTINATION "${CMAKE_INSTALL_PREFIX}" COMPONENT header)
