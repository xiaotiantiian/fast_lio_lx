#ifndef IMU_BSPLINE_UPDATER_H
#define IMU_BSPLINE_UPDATER_H

#include <Eigen/Dense>
#include <vector>
#include <deque>

namespace integral_bspline
{

    class IMUBsplineUpdater
    {
    public:
        struct IMUState
        {
            double timestamp;         // 时间戳
            Eigen::Matrix3d rotation; // 旋转矩阵(3x3)
            Eigen::Vector3d position; // 位置向量(3x1)
            Eigen::Vector3d velocity; // 速度向量(3x1)
        };
        // 使用三阶B样条
        IMUBsplineUpdater(int degree = 3);
        // 重置函数
        void reset();
        // 添加控制点(时间戳+旋转矩阵+平移向量)
        void addControlPoint(double timestamp, const Eigen::Matrix3d &R, const Eigen::Vector3d &T);
        // 基于B样条插值，更新IMU状态向量
        void updateIMUPoses(std::vector<IMUState> &imu_states);
        // 直接插值
        void interpolatePose(double timestamp, Eigen::Matrix3d &out_R, Eigen::Vector3d &out_T) const;
        void interpoint(double timestamp, Eigen::Matrix3d &out_R, Eigen::Vector3d &out_T) const;

        int degree_;

    private:
        std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> control_points_; // 选择的控制点 储存旋转矩阵和平移向量
        std::vector<double> timestamps_;                                          // 控制点时间戳
        // 生成B样条所需的节点向量
        std::vector<double> generateKnotVector() const;
        std::vector<double> generateclampedKnotVector() const;
    };

} // namespace integral_bspline

#endif // IMU_BSPLINE_UPDATER_H