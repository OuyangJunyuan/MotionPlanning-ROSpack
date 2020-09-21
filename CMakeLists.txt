############################################################################################
##                                          常用宏
##                                      PROJECT_NAME
############################################################################################
cmake_minimum_required(VERSION 3.0.2)
##需和xml文件中的name一致
project(MotionPlanning)
#add_compile_options(-std=c++11)

############################################################################################
##                                       ros packages
############################################################################################
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        std_msgs
        message_generation
        )

##添加 add_message_files	和 generate_messages 于下方
#add_message_files(
#        FILES
#        test_msg.msg
#)
#generate_messages(
#        DEPENDENCIES
#        #如果依赖于其他非原生类型则需要添加依赖如
#        std_msgs
#        geometry_msgs
#)

#一般可以不管他，只是为使其他依赖于本包的程序通过find_package就能获取本包内的所有依赖而不用重新添加(只能添加ROS包)。
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES mypackage
        #  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        #  DEPENDS system_lib
)

##开始指定路径,禁止在之后寻找ROS的库
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
############################################################################################
##                                      non-ros packages
## <name>_FOUND
## <name>_INCLUDE_DIRS
## <name>_LIBRARIES
############################################################################################
##                          当前目录下放的头文件
#####################################################
#添加头文件目录
include_directories("./include")  #相对于CMakeLists.txt
#添加库文件目录
link_directories(" ./lib")  #相对于执行CMake的目录或路径一般是在
#添加所有源文件名到SRC_DIRS中
#aux_source_directory(./src  SRCS_FILES)
#EXECUTABLE_OUTPUT_PATH(./bin)
#LIBRARY_OUTPUT_PATH(./lib)

#                          Eigen
include_directories("/usr/include/eigen3")

##                          EBoost
#find_package(Boost REQUIRED COMPONENTS
# system
#)

##                          OpenCV
#find_package(OpenCV  REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

############################################################################################
##                                       build
############################################################################################
##                     generate libs like .s .so
#####################################################
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mypackage.cpp
# )
##库内有用到自定义消息类型时启用
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#####################################################
##                          node
#####################################################
add_executable(${PROJECT_NAME}_node src/main.cpp  )
##可执行文件内有自定义消息类型时启用
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )

#####################################################
##                         non-node
#####################################################
#add_executable(${PROJECT_NAME}_node src/node.cpp)
#target_link_libraries(${PROJECT_NAME}_node
#        ${catkin_LIBRARIES}
#        )