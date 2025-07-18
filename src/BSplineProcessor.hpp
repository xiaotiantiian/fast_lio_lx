// BSplineProcessor.hpp
#ifndef BSPLINE_PROCESSOR_HPP
#define BSPLINE_PROCESSOR_HPP


#include <Eigen/Core>
#include "so3_math.h"
#include "imu_bspline_updater.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pointcloud_split.h"


class BSplineProcessor
{
public:
    BSplineProcessor();

    // 初始化参数
    void init(ros::NodeHandle &nh);

    // // 处理 B-spline 逻辑的总入口
    // void processBSpline(
    //     const double lidar_end_time,
    //     const state_ikfom &state_point,
    //     PointCloudXYZI::Ptr &feats_undistort,
    //     const PointCloudXYZI::Ptr &feats_undistort_prev,
    //     const state_ikfom &state_prev,
    //     bool flg_EKF_inited,
    //     int meas_num,
    //     state_ikfom &new_state_point,
    //     esekfom::esekf<state_ikfom, 12, input_ikfom> &kf,
    //     double &solve_H_time);

private:
    // B-spline 相关变量
    std::deque<state_ikfom> int_state_queue;
    std::deque<state_ikfom> mat_state_queue;
    std::deque<double> state_time_queue;
    std::deque<Eigen::Matrix3d> int_rot_queue;
    std::deque<Eigen::Vector3d> int_pos_queue;
    std::deque<Eigen::Matrix3d> mat_rot_queue;
    std::deque<Eigen::Vector3d> mat_pos_queue;
    Eigen::Matrix3d delta_rot;
    Eigen::Vector3d delta_pos;
    SO3 update_int_rot;
    Eigen::Vector3d update_int_pos;
    SO3 update_mat_rot;
    Eigen::Vector3d update_mat_pos;
    state_ikfom new_state_point;
    integral_bspline::IMUBsplineUpdater bspline_int;
    integral_bspline::IMUBsplineUpdater bspline_mat;

public:
    bool new_bsline_en;
    int state_queue_size;
    // 更新状态队列
    void updateintStateQueues(
        const double lidar_end_time,
        const state_ikfom &state_point);

    void updatematStateQueues(
        const double lidar_end_time,
        const state_ikfom &state_point);

    // B-spline 积分
    void computeBSplineIntegration();

    // B-spline 匹配
    void computeBSplineMatching();

    // 计算增量
    void computeDelta();

    // 更新状态并进行第二次去畸变
    void updateStateAndUndistort(
        const state_ikfom &state_point,
        PointCloudXYZI::Ptr &feats_undistort);

    // // 第二次 EKF 更新
    // void performSecondEKFUpdate(
    //     esekfom::esekf<state_ikfom, 12, input_ikfom> &kf,
    //     PointCloudXYZI::Ptr &feats_undistort,
    //     state_ikfom &state_point,
    //     double &solve_H_time);

    // 二次去畸变函数
    void secondUndistortCloud(
        const PointCloudXYZI::Ptr &feats_undistort,
        const state_ikfom &state_point,
        const state_ikfom &new_state_point);
};

#endif // BSPLINE_PROCESSOR_HPP
