#include "imu_bspline_updater.h"
#include <algorithm>
#include <array>
#include <Eigen/Geometry>
#include <iostream>

int num = 0;
int debug_counter = 0;

namespace integral_bspline
{

    IMUBsplineUpdater::IMUBsplineUpdater(int degree) : degree_(degree) {}

    void IMUBsplineUpdater::reset()
    {
        timestamps_.clear();
        control_points_.clear();
    }

    void IMUBsplineUpdater::addControlPoint(double timestamp, const Eigen::Matrix3d &R, const Eigen::Vector3d &T)
    {
        timestamps_.push_back(timestamp);
        control_points_.emplace_back(R, T);
    }

    // 均匀 clamped B样条（11个控制点相当于节点选择为：[0,0,0,0,1,2,3,4,5,6,7,10,10,10,10]）
    // 拟合出来的b样条函数会通过首尾  用于拟合单帧内部的imu状态
    std::vector<double> IMUBsplineUpdater::generateclampedKnotVector() const
    {
        if (timestamps_.empty())
            return {};

        std::vector<double> knots;
        int n = timestamps_.size();

        // 起始节点(degree+1个重复)
        knots.insert(knots.end(), degree_ + 1, timestamps_.front());
        // 中间节点(使用实际控制点时间戳)
        if (n > degree_ + 1)
        {
            for (int i = 1; i < n - degree_; ++i)
            {
                knots.push_back(timestamps_[i]);
            }
        }
        // 结束节点(degree+1个重复)
        knots.insert(knots.end(), degree_ + 1, timestamps_.back());
        return knots;
    }

    // 均匀 B样条
    std::vector<double> IMUBsplineUpdater::generateKnotVector() const
    {
        if (timestamps_.empty())
            return {};
        std::vector<double> knots;
        int n = timestamps_.size();          // 4
        double t0 = timestamps_.front();     // 0
        double t_last = timestamps_.back();  // 0.3
        double dt = (t_last - t0) / (n - 1); // 0.3 / (4-1) = 0.1
        // 生成均匀节点，长度为 n + degree_ + 1 = 4 + 3 + 1 = 8
        for (int i = 0; i < n + degree_ + 1; ++i)
        {
            knots.push_back(t0 + i * dt / (n - degree_)); // 均匀分布
        }
        return knots;
    }

    // std::vector<double> IMUBsplineUpdater::generateKnotVector() const
    // {
    //     if (timestamps_.empty())
    //         return {};

    //     std::vector<double> knots;
    //     size_t n = timestamps_.size(); // 控制点数量，使用 size_t 避免类型转换

    //     // 检查时间戳数量是否足够
    //     if (n < 2)
    //     {
    //         std::cerr << "[ERROR] Too few timestamps for knot vector generation" << std::endl;
    //         return {};
    //     }

    //     // 检查时间戳单调性
    //     for (size_t i = 1; i < n; ++i)
    //     {
    //         if (timestamps_[i] < timestamps_[i - 1])
    //         {
    //             std::cerr << "[ERROR] Timestamps must be non-decreasing" << std::endl;
    //             return {};
    //         }
    //     }

    //     // 获取首尾时间戳
    //     double t0 = timestamps_.front();
    //     double t_last = timestamps_.back();

    //     // 计算所需节点数
    //     size_t required_knots = n + degree_ + 1;

    //     // 添加首节点（重复 t0）
    //     size_t repeat_front = (required_knots - n + 1) / 2; // 首部重复次数
    //     knots.insert(knots.end(), repeat_front, t0);

    //     // 添加中间节点（直接使用 timestamps_）
    //     for (size_t i = 0; i < n; ++i)
    //     {
    //         knots.push_back(timestamps_[i]);
    //     }

    //     // 添加尾节点（重复 t_last）
    //     size_t current_knots = knots.size();
    //     size_t repeat_back = required_knots - current_knots; // 尾部重复次数
    //     knots.insert(knots.end(), repeat_back, t_last);

    //     // 调试输出（可选）
    //     // std::cout << "[DEBUG] Knot vector: ";
    //     // for (double k : knots) std::cout << k << " ";
    //     // std::cout << std::endl;

    //     return knots;
    // }
    // // 生成非均匀 clamped B样条节点向量
    // std::vector<double> IMUBsplineUpdater::generateKnotVector() const
    // {
    //     if (timestamps_.empty())
    //         return {};
    //     std::vector<double> knots;
    //     int n = timestamps_.size();          // 4
    //     double t0 = timestamps_.front();     // 0
    //     double t_last = timestamps_.back();  // 0.3
    //     double dt = (t_last - t0) / (n - 1); // 0.1
    //     // 首尾各一个节点，中间均匀分布
    //     knots.push_back(t0); // 1 次
    //     for (int i = 1; i <= n - 2; ++i)
    //     {
    //         knots.push_back(t0 + i * dt); // [0.1, 0.2]
    //     }
    //     knots.push_back(t_last); // 1 次
    //     // 补齐到 n + degree_ + 1 = 8
    //     knots.insert(knots.begin(), t0);   // [0, 0, 0.1, 0.2, 0.3]
    //     knots.insert(knots.end(), t_last); // [0, 0, 0.1, 0.2, 0.3, 0.3]
    //     knots.insert(knots.begin(), t0);   // [0, 0, 0, 0.1, 0.2, 0.3, 0.3]
    //     knots.insert(knots.end(), t_last); // [0, 0, 0, 0.1, 0.2, 0.3, 0.3, 0.3]
    //     return knots;
    // }
    // 二分查找确定参数 t 所在的节点区间
    template <typename T>
    constexpr T clamp(T value, T low, T high)
    {
        return std::min(high, std::max(low, value));
    }

    int findSpan(double t, int degree, const std::vector<double> &knots)
    {
        int low = degree;
        int high = knots.size() - degree - 1;

        while (high - low > 1)
        {
            int mid = (high + low) / 2;
            if (t < knots[mid])
            {
                high = mid;
            }
            else
            {
                low = mid;
            }
        }
        return low;
    }

    // Cox-de Boor递归算法，通过递归方式计算基函数值
    // 初始化 basis、left 和 right 向量
    std::vector<double> computeBasis(int span, double t, int degree, const std::vector<double> &knots)
    {
        std::vector<double> basis(degree + 1, 0.0);
        std::vector<double> left(degree + 1, 0.0);
        std::vector<double> right(degree + 1, 0.0);

        basis[0] = 1.0;

        for (int j = 1; j <= degree; ++j)
        {
            left[j] = t - knots[span + 1 - j];
            right[j] = knots[span + j] - t;
            double saved = 0.0;

            for (int r = 0; r < j; ++r)
            {
                double temp = basis[r] / (right[r + 1] + left[j - r]);
                basis[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            basis[j] = saved;
        }

        // for (int i = 0; i < basis.size(); i++)
        // {
        //     std::cout << "basis  " << i<< " : "<< basis[i] << std::endl;
        // }

        return basis;
    }

    // 内部
    void IMUBsplineUpdater::updateIMUPoses(std::vector<IMUState> &IMUpose)
    {
        if (control_points_.size() < degree_ + 1 || IMUpose.empty())
        {
            std::cout << "[DEBUG] Not enough control points or empty IMU states" << std::endl;
            return;
        }

        auto knots = generateclampedKnotVector();

        if (knots.size() < 2 * (degree_ + 1))
        {
            std::cout << "[DEBUG] Invalid knot vector size: " << knots.size() << std::endl;
            return;
        }

        for (auto &state : IMUpose)
        {
            double t = state.timestamp;
            t = clamp(t, knots[degree_], knots[knots.size() - degree_ - 1]);

            // std::cout << "\n[DEBUG] Processing IMU state at time: " << num << "  " << t << std::endl;

            // if (num == 20)
            // {
            //     num = -1;
            // }
            // num++;

            // 查找节点区间
            int span = findSpan(t, degree_, knots);

            // std::cout << "[DEBUG] Found span: " << span << std::endl;

            // 计算B样条基函数
            auto basis = computeBasis(span, t, degree_, knots);

            // 打印基函数值
            // std::cout << "[DEBUG] Basis weights: ";
            // for (auto w : basis) std::cout << w << " ";
            // std::cout << std::endl;

            // 广义四元数插值
            Eigen::Quaterniond q_result = Eigen::Quaterniond::Identity(); // 初始化结果四元数为单位四元数
            Eigen::Vector3d log_sum = Eigen::Vector3d::Zero();            // 初始化对数空间旋转累积向量为零向量
            double total_weight = 0.0;

            // std::cout << "[DEBUG] Control points used for rotation:" << std::endl;

            for (int i = 0; i <= degree_; ++i)
            {
                int idx = span - degree_ + i;
                if (idx >= 0 && idx < control_points_.size() && basis[i] > 1e-6)
                {
                    Eigen::Quaterniond q_control(control_points_[idx].first); // 控制点mat-->quat
                    q_control.normalize();                                    // 归一化

                    // std::cout << "  [CP " << idx << "] Q: " << q_control.w() << " "
                    //           << q_control.x() << " " << q_control.y() << " " << q_control.z()
                    //           << " (weight: " << basis[i] << ")" << std::endl;

                    if (total_weight == 0.0)
                    {
                        q_result = q_control;
                        total_weight += basis[i];
                        continue;
                    }

                    Eigen::Quaterniond delta_q = q_result.conjugate() * q_control; // 计算当前结果到控制点的相对旋转（四元数乘法）
                    Eigen::AngleAxisd delta_aa(delta_q);                           // 将相对旋转转换为轴角表示
                    log_sum += basis[i] * delta_aa.axis() * delta_aa.angle();      // 将对数空间旋转（轴*角度）按权重累加
                    total_weight += basis[i];
                }
            }

            // // 打印对数空间累积值
            // std::cout << "[DEBUG] Log-space sum: " << log_sum.transpose()
            //           << " (total weight: " << total_weight << ")" << std::endl;

            // 应用累积的旋转增量
            if (total_weight > 1e-6 && log_sum.norm() > 1e-6)
            {
                double angle = log_sum.norm();               // 旋转向量的模表示旋转角度
                Eigen::Vector3d axis = log_sum.normalized(); // 单位旋转轴
                Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
                q_result = q_result * delta_q;
                q_result.normalize();

                // std::cout << "[DEBUG] Applied delta rotation - angle: " << angle
                //           << ", axis: " << axis.transpose() << std::endl;
            }

            // //打印插值前的四元数
            // Eigen::Quaterniond q_before(state.rotation);
            // std::cout << "[DEBUG] Before interpolation Q:" << std::endl;
            // std::cout << q_before.w() << " " << q_before.x() << " " << q_before.y() << " " << q_before.z() << std::endl;
            // // 打印插值后的四元数
            // std::cout << "[DEBUG] After interpolation Q:" << std::endl;
            // std::cout << q_result.w() << " " << q_result.x() << " " << q_result.y() << " " << q_result.z() << std::endl;

            // 平移插值
            Eigen::Vector3d new_T = Eigen::Vector3d::Zero();
            double weight_sum = 0.0;

            // std::cout << "[DEBUG] Control points used for translation:" << std::endl;
            for (int i = 0; i <= degree_; ++i)
            {
                int idx = span - degree_ + i;
                if (idx >= 0 && idx < control_points_.size())
                {
                    double weight = basis[i];
                    new_T += weight * control_points_[idx].second;
                    weight_sum += weight;

                    //         std::cout << "  [CP " << idx << "] T: " << control_points_[idx].second.transpose()
                    //                   << " (weight: " << weight << ")" << std::endl;
                }
            }

            if (weight_sum > 1e-6)
            {
                new_T /= weight_sum;
            }

            // std::cout << "[DEBUG] Before interpolation r:" << std::endl;
            // std::cout << state.rotation << std::endl;
            // std::cout << "[DEBUG] Interpolated translation: " << new_T.transpose() << std::endl;

            // 更新状态
            state.rotation = q_result.toRotationMatrix();
            state.position = new_T;

            // std::cout << "[DEBUG] After interpolation r:" << std::endl;
            // std::cout << state.rotation << std::endl;
        }
    }

    // 外部
    void IMUBsplineUpdater::interpolatePose(double timestamp, Eigen::Matrix3d &out_R, Eigen::Vector3d &out_T) const
    {
        // 默认值（单位旋转 + 零平移）
        out_R = Eigen::Matrix3d::Identity();
        out_T = Eigen::Vector3d::Zero();

        // 检查控制点数量是否足够
        if (control_points_.size() < degree_ + 1)
        {
            return;
        }

        auto knots = generateKnotVector();
        timestamp = clamp(timestamp, knots[degree_], knots[knots.size() - degree_ - 1]);

        // 查找节点区间
        int span = findSpan(timestamp, degree_, knots);

        // 计算B样条基函数
        auto basis = computeBasis(span, timestamp, degree_, knots);

        // 旋转插值（四元数球面线性插值）
        Eigen::Quaterniond q_result = Eigen::Quaterniond::Identity();
        Eigen::Vector3d log_sum = Eigen::Vector3d::Zero();
        double total_weight = 0.0;

        for (int i = 0; i <= degree_; ++i)
        {
            int idx = span - degree_ + i;
            if (idx >= 0 && idx < control_points_.size() && basis[i] > 1e-6)
            {
                Eigen::Quaterniond q_control(control_points_[idx].first);
                q_control.normalize();

                if (total_weight == 0.0)
                {
                    q_result = q_control;
                    total_weight += basis[i];
                    continue;
                }

                Eigen::Quaterniond delta_q = q_result.conjugate() * q_control;
                Eigen::AngleAxisd delta_aa(delta_q);
                log_sum += basis[i] * delta_aa.axis() * delta_aa.angle();
                total_weight += basis[i];
            }
        }

        if (total_weight > 1e-6 && log_sum.norm() > 1e-6)
        {
            double angle = log_sum.norm();
            Eigen::Vector3d axis = log_sum.normalized();
            Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
            q_result = q_result * delta_q;
            q_result.normalize();
        }
        // 改进后的旋转插值部分
        // Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
        // Eigen::Vector3d log_sum = Eigen::Vector3d::Zero();
        // double total_weight = 0.0;

        // // 首先找到一个参考旋转（可以选择第一个非零权重的控制点）
        // for (int i = 0; i <= degree_; ++i)
        // {
        //     int idx = span - degree_ + i;
        //     if (idx >= 0 && idx < control_points_.size() && basis[i] > 1e-6)
        //     {
        //         q_ref = Eigen::Quaterniond(control_points_[idx].first).normalized();
        //         break;
        //     }
        // }

        // // 计算相对于参考旋转的对数空间增量
        // for (int i = 0; i <= degree_; ++i)
        // {
        //     int idx = span - degree_ + i;
        //     if (idx >= 0 && idx < control_points_.size() && basis[i] > 1e-6)
        //     {
        //         Eigen::Quaterniond q_control(control_points_[idx].first);
        //         q_control.normalize();

        //         Eigen::Quaterniond delta_q = q_ref.conjugate() * q_control;
        //         Eigen::AngleAxisd delta_aa(delta_q);
        //         log_sum += basis[i] * delta_aa.axis() * delta_aa.angle();
        //         total_weight += basis[i];
        //     }
        // }

        // // 应用插值结果
        // if (total_weight > 1e-6)
        // {
        //     if (log_sum.norm() > 1e-6)
        //     {
        //         double angle = log_sum.norm() / total_weight; // 加权平均
        //         Eigen::Vector3d axis = log_sum.normalized();
        //         Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
        //         q_result = q_ref * delta_q;
        //     }
        //     else
        //     {
        //         q_result = q_ref; // 很小的旋转变化
        //     }
        //     q_result.normalize();
        // }
        out_R = q_result.toRotationMatrix();

        // 平移插值
        out_T = Eigen::Vector3d::Zero();
        double weight_sum = 0.0;
        for (int i = 0; i <= degree_; ++i)
        {
            int idx = span - degree_ + i;
            if (idx >= 0 && idx < control_points_.size())
            {
                out_T += basis[i] * control_points_[idx].second;
                weight_sum += basis[i];
            }
        }
        if (weight_sum > 1e-6)
        {
            out_T /= weight_sum;
        }
    }

    void IMUBsplineUpdater::interpoint(double timestamp, Eigen::Matrix3d &out_R, Eigen::Vector3d &out_T) const
    {
        // 默认值（单位旋转 + 零平移）
        out_R = Eigen::Matrix3d::Identity();
        out_T = Eigen::Vector3d::Zero();

        // 检查控制点数量是否足够
        if (control_points_.size() < degree_ + 1)
        {
            return;
        }

        auto knots = generateclampedKnotVector();
        timestamp = clamp(timestamp, knots[degree_], knots[knots.size() - degree_ - 1]);

        // 查找节点区间
        int span = findSpan(timestamp, degree_, knots);

        // 计算B样条基函数
        auto basis = computeBasis(span, timestamp, degree_, knots);

        // 旋转插值（四元数球面线性插值）
        Eigen::Quaterniond q_result = Eigen::Quaterniond::Identity();
        Eigen::Vector3d log_sum = Eigen::Vector3d::Zero();
        double total_weight = 0.0;

        for (int i = 0; i <= degree_; ++i)
        {
            int idx = span - degree_ + i;
            if (idx >= 0 && idx < control_points_.size() && basis[i] > 1e-6)
            {
                Eigen::Quaterniond q_control(control_points_[idx].first);
                q_control.normalize();

                if (total_weight == 0.0)
                {
                    q_result = q_control;
                    total_weight += basis[i];
                    continue;
                }

                Eigen::Quaterniond delta_q = q_result.conjugate() * q_control;
                Eigen::AngleAxisd delta_aa(delta_q);
                log_sum += basis[i] * delta_aa.axis() * delta_aa.angle();
                total_weight += basis[i];
            }
        }

        if (total_weight > 1e-6 && log_sum.norm() > 1e-6)
        {
            double angle = log_sum.norm();
            Eigen::Vector3d axis = log_sum.normalized();
            Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
            q_result = q_result * delta_q;
            q_result.normalize();
        }
        // 改进后的旋转插值部分
        // Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
        // Eigen::Vector3d log_sum = Eigen::Vector3d::Zero();
        // double total_weight = 0.0;

        // // 首先找到一个参考旋转（可以选择第一个非零权重的控制点）
        // for (int i = 0; i <= degree_; ++i)
        // {
        //     int idx = span - degree_ + i;
        //     if (idx >= 0 && idx < control_points_.size() && basis[i] > 1e-6)
        //     {
        //         q_ref = Eigen::Quaterniond(control_points_[idx].first).normalized();
        //         break;
        //     }
        // }

        // // 计算相对于参考旋转的对数空间增量
        // for (int i = 0; i <= degree_; ++i)
        // {
        //     int idx = span - degree_ + i;
        //     if (idx >= 0 && idx < control_points_.size() && basis[i] > 1e-6)
        //     {
        //         Eigen::Quaterniond q_control(control_points_[idx].first);
        //         q_control.normalize();

        //         Eigen::Quaterniond delta_q = q_ref.conjugate() * q_control;
        //         Eigen::AngleAxisd delta_aa(delta_q);
        //         log_sum += basis[i] * delta_aa.axis() * delta_aa.angle();
        //         total_weight += basis[i];
        //     }
        // }

        // // 应用插值结果
        // if (total_weight > 1e-6)
        // {
        //     if (log_sum.norm() > 1e-6)
        //     {
        //         double angle = log_sum.norm() / total_weight; // 加权平均
        //         Eigen::Vector3d axis = log_sum.normalized();
        //         Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
        //         q_result = q_ref * delta_q;
        //     }
        //     else
        //     {
        //         q_result = q_ref; // 很小的旋转变化
        //     }
        //     q_result.normalize();
        // }
        out_R = q_result.toRotationMatrix();

        // 平移插值
        out_T = Eigen::Vector3d::Zero();
        double weight_sum = 0.0;
        for (int i = 0; i <= degree_; ++i)
        {
            int idx = span - degree_ + i;
            if (idx >= 0 && idx < control_points_.size())
            {
                out_T += basis[i] * control_points_[idx].second;
                weight_sum += basis[i];
            }
        }
        if (weight_sum > 1e-6)
        {
            out_T /= weight_sum;
        }
    }
} // namespace integral_bspline