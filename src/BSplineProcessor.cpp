#include "BSplineProcessor.hpp"
#include <ros/ros.h>
#include <omp.h>

BSplineProcessor::BSplineProcessor() : new_bsline_en(false), state_queue_size(0) {}

void BSplineProcessor::updateintStateQueues(
    const double lidar_end_time,
    const state_ikfom &state_point)
{
    // 更新状态时间队列
    state_time_queue.push_back(lidar_end_time);
    if (state_time_queue.size() > state_queue_size)
    {
        state_time_queue.pop_front();
    }

    // 更新积分旋转和位置队列
    int_rot_queue.push_back(state_point.rot.toRotationMatrix());
    if (int_rot_queue.size() > state_queue_size)
    {
        int_rot_queue.pop_front();
    }
    int_pos_queue.push_back(state_point.pos);
    if (int_pos_queue.size() > state_queue_size)
    {
        int_pos_queue.pop_front();
    }
}

void BSplineProcessor::updatematStateQueues(
    const double lidar_end_time,
    const state_ikfom &state_point)
{

    // 更新匹配旋转和位置队列
    mat_rot_queue.push_back(state_point.rot.toRotationMatrix());
    if (mat_rot_queue.size() > state_queue_size)
    {
        mat_rot_queue.pop_front();
    }
    mat_pos_queue.push_back(state_point.pos);
    if (mat_pos_queue.size() > state_queue_size)
    {
        mat_pos_queue.pop_front();
    }
}

void BSplineProcessor::computeBSplineIntegration()
{
    if (state_time_queue.size() != state_queue_size)
    {
        return;
    }

    bspline_int.reset();                  // 清空之前的控制点
    double t0 = state_time_queue.front(); // 获取第一个时间戳
    for (int i = 0; i < state_queue_size; i++)
    {
        double relative_time = state_time_queue[i] - t0; // 转换为相对时间
        bspline_int.addControlPoint(
            relative_time,
            int_rot_queue[i],
            int_pos_queue[i]);
    }

    // 获取最后一个控制点的相对时间
    double last_relative_time = state_time_queue.back() - t0;
    // 调用插值函数
    Eigen::Matrix3d out_R;
    Eigen::Vector3d out_T;
    bspline_int.interpolatePose(last_relative_time, out_R, out_T);
    // 存储插值结果
    update_int_rot = SO3(out_R);
    update_int_pos = out_T;
}

void BSplineProcessor::computeBSplineMatching()
{
    if (state_time_queue.size() != state_queue_size)
    {
        return;
    }

    bspline_mat.reset();                  // 清空之前的控制点
    double t0 = state_time_queue.front(); // 获取第一个时间戳
    for (int i = 0; i < state_queue_size; i++)
    {
        double relative_time = state_time_queue[i] - t0; // 转换为相对时间
        bspline_mat.addControlPoint(
            relative_time,
            mat_rot_queue[i],
            mat_pos_queue[i]);
    }

    // 获取最后一个控制点的相对时间
    double last_relative_time = state_time_queue.back() - t0;
    // 调用插值函数
    Eigen::Matrix3d out_R;
    Eigen::Vector3d out_T;
    bspline_mat.interpolatePose(last_relative_time, out_R, out_T);
    // 存储插值结果
    update_mat_rot = SO3(out_R);
    update_mat_pos = out_T;
}

void BSplineProcessor::computeDelta()
{
    delta_rot = update_mat_rot * update_int_rot.inverse();
    delta_pos = update_mat_pos - delta_rot * update_int_pos;
}

void BSplineProcessor::updateStateAndUndistort(
    const state_ikfom &state_point,
    PointCloudXYZI::Ptr &feats_undistort)
{
    // 更新状态
    new_state_point = state_point;
    new_state_point.rot = delta_rot * new_state_point.rot;
    new_state_point.pos = delta_rot * new_state_point.pos + delta_pos;

    // 第二次去畸变
    secondUndistortCloud(feats_undistort, state_point, new_state_point);
}

// void BSplineProcessor::performSecondEKFUpdate(
//     esekfom::esekf<state_ikfom, 12, input_ikfom>& kf,
//     PointCloudXYZI::Ptr& feats_undistort,
//     state_ikfom& state_point,
//     double& solve_H_time) {
//     // 更新状态
//     state_point = new_state_point;
//     pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

//     if (feats_undistort->empty() || !feats_undistort) {
//         ROS_WARN("No point, skip this scan!\n");
//         return;
//     }

//     // 重新执行地图分割和降采样
//     lasermap_fov_segment();
//     downSizeFilterSurf.setInputCloud(feats_undistort);
//     downSizeFilterSurf.filter(*feats_down_body);
//     feats_down_size = feats_down_body->points.size();

//     // 初始化地图 kdtree
//     if (ikdtree.Root_Node == nullptr) {
//         if (feats_down_size > 5) {
//             ikdtree.set_downsample_param(filter_size_map_min);
//             feats_down_world->resize(feats_down_size);
//             for (int i = 0; i < feats_down_size; i++) {
//                 pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
//             }
//             ikdtree.Build(feats_down_world->points);
//         }
//         return;
//     }

//     normvec->resize(feats_down_size);
//     feats_down_world->resize(feats_down_size);

//     // 第二次 EKF 更新
//     double t_update_start = omp_get_wtime();
//     kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
//     state_point = kf.get_x();
// }

void BSplineProcessor::secondUndistortCloud(
    const PointCloudXYZI::Ptr &feats_undistort,
    const state_ikfom &state_point,
    const state_ikfom &new_state_point)
{
    // 提取外参和状态
    Eigen::Matrix3d R_LI = state_point.offset_R_L_I.toRotationMatrix();
    Eigen::Vector3d t_LI = state_point.offset_T_L_I;
    Eigen::Matrix3d R_1 = state_point.rot.toRotationMatrix();
    Eigen::Vector3d t_1 = state_point.pos;
    Eigen::Matrix3d R_2 = new_state_point.rot.toRotationMatrix();
    Eigen::Vector3d t_2 = new_state_point.pos;

    // 确保 feats_undistort 非空且大小匹配
    if (!feats_undistort || feats_undistort->empty())
    {
        std::cerr << "Error: Input point cloud is empty!" << std::endl;
        return;
    }

    // 遍历点云并更新坐标
    for (size_t i = 0; i < feats_undistort->points.size(); ++i)
    {
        auto &point = feats_undistort->points[i];

        // 第一次去畸变后的点 (p_L1)
        Eigen::Vector3d p_L1(point.x, point.y, point.z);

        // 公式：p_L2 = R_LI^T * [R_2^T * (R_1 * (R_LI * p_L1 + t_LI) + t_1 - t_2) - t_LI]
        // 步骤 1：转换到世界坐标系 p_w = R_1 * (R_LI * p_L1 + t_LI) + t_1
        Eigen::Vector3d p_w = R_1 * (R_LI * p_L1 + t_LI) + t_1;

        // 步骤 2：转换到更新后的 IMU 坐标系 p_I2 = R_2^T * (p_w - t_2)
        Eigen::Vector3d p_I2 = R_2.transpose() * (p_w - t_2);

        // 步骤 3：转换回 LiDAR 坐标系 p_L2 = R_LI^T * (p_I2 - t_LI)
        Eigen::Vector3d p_L2 = R_LI.transpose() * (p_I2 - t_LI);

        // 更新点云坐标（保留其他属性如强度）
        point.x = static_cast<float>(p_L2.x());
        point.y = static_cast<float>(p_L2.y());
        point.z = static_cast<float>(p_L2.z());
    }
}
