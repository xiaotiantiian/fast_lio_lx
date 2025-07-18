#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <deque>
#include <algorithm>
#include <iostream>
#include "use-ikfom.hpp"
#include "common_lib.h"

namespace tian
{
    class Lidar_processing
    {
    public:
        Lidar_processing() = default;
        ~Lidar_processing() = default;

        // int sweep_mode = 0;
        // int conbined_mode = 0;

    private:
        int pop_point = 0;
        int end_point_time = 0;
        int meas_num = 0;
        using PointType = pcl::PointXYZINormal;
        using PointCloudXYZI = pcl::PointCloud<PointType>;

    public:
        void splitByTime(
            const PointCloudXYZI::Ptr &input_cloud,
            double msg_timestamp,
            int num_splits,
            std::deque<PointCloudXYZI::Ptr> &lidar_buffer,
            std::deque<double> &time_buffer);

        void splitByCount(
            const PointCloudXYZI::Ptr &input_cloud,
            double msg_timestamp,
            int nun_splits,
            std::deque<PointCloudXYZI::Ptr> &lidar_buffer,
            std::deque<double> &time_buffer);

        void fuseClouds(
            const PointCloudXYZI::Ptr &feats_undistort,
            const PointCloudXYZI::Ptr &feats_undistort_prev,
            const state_ikfom &state_point,
            const state_ikfom &state_prev,
            PointCloudXYZI::Ptr &feats_combined);
    };
} // namespace tian
