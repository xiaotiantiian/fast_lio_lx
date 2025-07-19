cmake_minimum_required(VERSION 2.8.3) # 最低CMake版本要求
project(fast_lio) # 项目名称

SET(CMAKE_BUILD_TYPE "Debug") # 设置编译类型为Debug

ADD_COMPILE_OPTIONS(-std=c++14) # 添加编译选项 设置C++标准为C++14
set(CMAKE_CXX_FLAGS "-std=c++14 -O3") # 强制C++14标准并启用O3优化

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")


message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    #  add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3) # 定义宏启用多线程
    message("core for MP: 3")
  elseif(N GREATER 3)
    #  add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()


# message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
# add_definitions(-DMP_PROC_NUM=1)
# message("Forced single-thread mode: MP_PROC_NUM=1")

add_compile_options(-ffloat-store)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED) # 查找Python库
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS # ROS依赖包
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED) # 线性代数库 声明项目依赖的外部库，REQUIRED 表示找不到时报错。
find_package(PCL 1.8 REQUIRED) # 点云库

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS} # 包含catkin的头文件目录
  ${EIGEN3_INCLUDE_DIR} # 包含Eigen3的头文件目录
  ${PCL_INCLUDE_DIRS} # 包含PCL的头文件目录
  ${PYTHON_INCLUDE_DIRS} # 包含Python的头文件目录
  include) # 包含项目的头文件目录

add_message_files( # 定义自定义消息
  FILES
  Pose6D.msg
)

generate_messages( # 生成消息代码
  DEPENDENCIES
  geometry_msgs
)

catkin_package( # 声明ROS包的导出规则
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_library(bag_reader # 将ros_bag_reader.cpp编译为库
  src/ros_bag_reader.cpp
)

add_library(pointcloud_split
  src/pointcloud_split.cpp
)

# 添加B样条优化库
add_library(imu_bspline_updater
  src/imu_bspline_updater.cpp
)

add_library(BSplineProcessor
  src/BSplineProcessor.cpp
)


add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp) # 主可执行文件
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} bag_reader pointcloud_split imu_bspline_updater BSplineProcessor ${PCL_LIBRARIES} ${PYTHON_LIBRARIES}) # 链接依赖库
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
