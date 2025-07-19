##  重力对齐

1. 在 IMU_Processing.hpp 文件的 IMU_init 函数中添加重力对齐代码，实现世界坐标系的摆正。

## 状态量日志

1. 在 IMU_Processing.hpp 文件 ImuProcess 类中，添加 save_imu_state 函数，用来保存每帧lidar尾的timestamp、pos和rot；

##  读包功能 

1. 在 mid360.yaml 文件中添加配置参数，方便做调试：

    - read_mode: 1   # 1表示读包 0表示播包
    - play_times: 1  # 读包的倍速
    - bag_num: 1     # 读取包的序列号 与下面的bag_names对应
    - bag_names: ["2024-07-26-17-11-38.bag","1_2024-08-26-16-33-17.bag"] #读取包的名字
    - bag_file_path ："/home/tian/workspace/fast_lio_raw/bag/" # 包所在的路径

2. 在 laserMapping.cpp 文件 添加头文件 #include "ros_bag_reader.h"

3. 在 laserMapping.cpp 文件 添加上面的配置参数，定义和调用param赋值。

4. 在 laserMapping.cpp 文件 添加 bag_reader 来使用读包功能。

    （具体参照ros_bag_reader.h 和 ros_bag_reader.cpp）

5. 在 CMakeLists.txt 文件 添加 

    add_library(bag_reader src/ros_bag_reader.cpp)

    使 ros_bag_reader 的功能可以被其他目标（如 fastlio_mapping）复用

## 点云分割重组
1. 在 mid360.yaml 文件中添加配置参数，方便做调试
   -  sweep_mode: 2           # 0 不分割 1 按时间分割 2 按数量分割
   -  conbined_mode: 1        # 0 不启动点云复用 单独分割  1 启动点云复用 坐标转换

2. 在 laserMapping.cpp 文件 添加上面的配置参数，定义和调用param赋值。

3. 在 laserMapping.cpp 文件 添加 splitByTime 函数 和 splitByCount 函数用于分割点云。

    并写入 livox_pcl_cbk 函数中，按模式选择分割类型。

4. 在 laserMapping.cpp 文件 添加 fuseClouds 函数，用于复用点云。

    并在 main 函数中执行 process 函数之后添加 fuseClouds 函数，根据模式选择是否复用点云。

5. 在 laserMapping.cpp 文件的 publish_frame_world 函数中根据是否复用点云，选择存入pcd文件点云的数量（复用点云需要减半）。

6. 分离为 poindcloud_split.h 和 pointcloud_split.cpp 文件。

## sensor_msgs/PointCloud2 点云格式的回调函数
1. 在 proprocess.cpp 文件中添加 avia_handler2() 函数，用于处理 sensor_msgs/PointCloud2 点云格式的lidar数据。

2. 在 laserMapping.cpp 文件中修改  livox_pcl_cbk 函数的参数类型。

3. 若和读包功能一起，还需要修改几处相关回调函数的参数类型。#

4. 已增加 lidar_rostopic 参数来选择使用哪种点云格式。


## ikd-tree双线程关闭
1. 在头文件中修改 ikd-tree.cpp 为单线程，以解决数据异常的情况。



