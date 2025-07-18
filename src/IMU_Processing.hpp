#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"
#include "preprocess.h"

#include "imu_bspline_updater.h"

#include <iostream>
#include <iomanip>
#include <boost/array.hpp>
/// *************Preconfiguration

#define MAX_INI_COUNT (10)

// 积分B样条
// MAT_TO_ARRAY VEC_TO_ARRAY 宏定义
#define MAT_TO_ARRAY(src, dst) \
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>((dst).data()) = (src);

#define VEC_TO_ARRAY(src, dst) \
  Eigen::Map<Eigen::Vector3d>((dst).data()) = (src).eval();

const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

/***readparam ***/
int read_mode, bag_num, lidar_rostopic;
int state_queue_size;

int bs_integration;
int point_degree;
int control_state_num;
/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void save_imu_state(double timestamp, const V3D &pos, const SO3 &rot);

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int lidar_type;
  int imu_state_num = 0;

private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  // 积分B样条
  void applyBSplineCorrection(state_ikfom &imu_state,
                              std::vector<Pose6D> &IMUpose,
                              double lidar_end_time);
  integral_bspline::IMUBsplineUpdater imu_bspline_updater_;

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D> v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

// 积分B样条
void ImuProcess::applyBSplineCorrection(state_ikfom &imu_state,
                                        std::vector<Pose6D> &IMUpose,
                                        double lidar_end_time)
{
  imu_bspline_updater_.degree_ = point_degree;
  cout << imu_bspline_updater_.degree_ << endl;
  if (!IMUpose.empty())
  {
    const int num_points = IMUpose.size();
    // const int num_control_points = 11;                            // 要选取的点数
    const int num_control_points = control_state_num;             // 要选取的点数
    const int step = (num_points - 1) / (num_control_points - 1); // 计算步长

    for (int i = 0; i < num_control_points; ++i)
    {
      int idx = (i == num_control_points - 1) ? (num_points - 1) : (i * step); // 确保最后一个点取到末尾
      M3D R;
      R << MAT_FROM_ARRAY(IMUpose[idx].rot);
      V3D T;
      T << VEC_FROM_ARRAY(IMUpose[idx].pos);
      imu_bspline_updater_.addControlPoint(IMUpose[idx].offset_time, R, T);
      // std::cout << "offset_time: " << std::fixed << std::setprecision(6) << IMUpose[idx].offset_time << "\n";
    }
  }

  // 准备IMU状态数据
  std::vector<integral_bspline::IMUBsplineUpdater::IMUState> states;
  for (const auto &pose : IMUpose)
  {
    integral_bspline::IMUBsplineUpdater::IMUState state;
    state.timestamp = pose.offset_time;
    state.rotation << MAT_FROM_ARRAY(pose.rot);
    state.position << VEC_FROM_ARRAY(pose.pos);
    // state.velocity << VEC_FROM_ARRAY(pose.vel);
    states.push_back(state);
  }
  imu_bspline_updater_.updateIMUPoses(states);

  // 更新IMUpose (注意：这里只更新旋转和平移)
  for (int i = 0; i < IMUpose.size() && i < states.size(); ++i)
  {
    MAT_TO_ARRAY(states[i].rotation, IMUpose[i].rot);
    VEC_TO_ARRAY(states[i].position, IMUpose[i].pos);
  }
}

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::save_imu_state(double timestamp, const V3D &pos, const SO3 &rot)
{
  // 构建目录路径
  std::stringstream dir_ss;
  // dir_ss << "/home/tian/workspace/fast_lio_bspline_new2/bs/"
  //        << state_queue_size << "/";
  dir_ss << string(ROOT_DIR) << "bs/"
         << state_queue_size << "/";

  // 创建目录（如果不存在）
  std::string dir_path = dir_ss.str();
  struct stat info;
  if (stat(dir_path.c_str(), &info) != 0)
  {
    std::string cmd = "mkdir -p " + dir_path;
    if (system(cmd.c_str()) != 0)
    {
      std::cerr << "Error creating directory: " << dir_path << std::endl;
      return;
    }
  }

  // 构建文件路径
  std::stringstream file_ss;
  file_ss << dir_path << bag_num << "_" << state_queue_size << ".txt";

  // 写入数据
  std::ofstream outfile(file_ss.str(), std::ios::app);
  if (outfile.is_open())
  {
    outfile << std::fixed << std::setprecision(6)
            // << timestamp << " "
            // << pos(0) << " " << pos(1) << " " << pos(2) << " "
            << pos.norm() << "\n";
    cout << std::fixed << std::setprecision(6)
         << timestamp << " "
         << rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w() << " "
         << pos(0) << " " << pos(1) << " " << pos(2) << " "
         << pos.norm() << "\n";
    // std::cout << "Data saved to: " << file_ss.str() << std::endl;
  }
  else
  {
    std::cerr << "Error opening file: " << file_ss.str() << std::endl;
  }
}

// void ImuProcess::save_imu_state(double timestamp, const V3D &pos, const SO3 &rot)
// {
//    std::string file_path = "/home/tian/workspace/fast_lio_raw/imu_trajectory.txt";
//   // 打开文件（追加模式）
//   std::ofstream outfile(dir_path, std::ios::app);
//   if (outfile.is_open())
//   {
//     // 将 SO3 转换为四元数
//     Eigen::Quaterniond q(rot.matrix());

//     std::cout << "acc_cov: " << acc_cov << " " << " gyr_cov: " << gyr_cov << "\n"
//               << std::fixed << std::setprecision(6)
//               << timestamp << "\n"
//               << pos.transpose() << " " << pos.norm() << "\n"
//               // << q.coeffs().transpose() << "\n"
//               << std::endl;

//     outfile << "acc_cov: " << acc_cov << " " << " gyr_cov: " << gyr_cov << "\n"
//             << std::fixed << std::setprecision(6)
//             << timestamp << " "
//             << pos(0) << " " << pos(1) << " " << pos(2) << " "
//             << pos.norm() << "\n";

//     // << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
//   }
// }

void ImuProcess::Reset()
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

int imu_init_times = 0;

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);

  /* 重力对齐 */
  // 计算重力对齐的旋转矩阵
  cout << "acc = " << mean_acc[0] << "  " << mean_acc[1] << "  " << mean_acc[2] << endl;
  V3D gravity_imu = -mean_acc.normalized(); // IMU坐标系下的重力方向（单位向量）
  V3D gravity_world(0, 0, -1);              // 目标重力方向（世界坐标系Z轴向下）

  // 计算旋转矩阵，将重力方向从IMU坐标系对齐到世界坐标系
  Eigen::Matrix3d R_wi = Eigen::Quaterniond::FromTwoVectors(gravity_imu, gravity_world).toRotationMatrix();
  cout << "R_wi = \n"
       << R_wi << endl;

  init_state.rot = R_wi;                                             // 覆盖初始姿态的旋转部分
  init_state.grav = S2(-R_wi * mean_acc / mean_acc.norm() * G_m_s2); // 更新grav 这个不更新的话 imu初始会有漂移
  /* 重力对齐 */

  // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = 0.00001;
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

  imu_init_times++;
  // std::cout << "imu_init_times: " << imu_init_times << std::endl;
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();

  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;

  if (lidar_type == MARSIM)
  {
    pcl_beg_time = last_lidar_end_time_;
    pcl_end_time = meas.lidar_beg_time;
  }

  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  // std::cout << "un_undistort_points_num:" << meas.lidar->points.size() << std::endl;

  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  cout << "[ IMU Process ]: Process lidar from " << pcl_beg_time << " to " << pcl_end_time << ", "
       << meas.imu.size() << " imu msgs from " << imu_beg_time << " to " << imu_end_time << endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if (head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  for (int i = 0; i < IMUpose.size(); i++)
    std::cout << "offset_time: " << std::fixed << std::setprecision(6) << IMUpose[i].offset_time << "\n";

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();

  imu_state_num++;
  std::cout << "imu_state_num: " << imu_state_num << std::endl;

  // save_imu_state(pcl_end_time, imu_state.pos, imu_state.rot);

  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  if (bs_integration == 1)
  {
    // B样条初始化并进行B样条优化
    imu_bspline_updater_.reset();
    applyBSplineCorrection(imu_state, IMUpose, pcl_end_time);

    /*** undistort each lidar point (backward propagation) ***/

    if (pcl_out.points.empty())
      return;

    // 从后向前处理点云
    for (auto it_pcl = pcl_out.points.end() - 1; it_pcl != pcl_out.points.begin(); it_pcl--)
    {
      // 计算当前点的时间偏移（相对于lidar_beg_time）
      double point_time = it_pcl->curvature / 1000.0;

      // 使用B样条插值获取位姿
      M3D R_i;
      V3D T_i;
      imu_bspline_updater_.interpoint(point_time, R_i, T_i); // B样条

      // 运动补偿公式
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() *
                         (imu_state.rot.conjugate() *
                          (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) +
                           (T_i - imu_state.pos) -
                           imu_state.offset_T_L_I));

      // 更新点坐标
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin())
        break;
    }
  }
  else
  {
    if (pcl_out.points.begin() == pcl_out.points.end())
      return;

    if (lidar_type != MARSIM)
    {
      auto it_pcl = pcl_out.points.end() - 1;
      for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
      {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        acc_imu << VEC_FROM_ARRAY(tail->acc);
        angvel_avr << VEC_FROM_ARRAY(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
        {
          dt = it_pcl->curvature / double(1000) - head->offset_time;

          /* Transform to the 'end' frame, using only the rotation
           * Note: Compensation direction is INVERSE of Frame's moving direction
           * So if we want to compensate a point at timestamp-i to the frame-e
           * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
          M3D R_i(R_imu * Exp(angvel_avr, dt));

          V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
          V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
          V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I); // not accurate!

          // save Undistorted points and their rotation
          it_pcl->x = P_compensate(0);
          it_pcl->y = P_compensate(1);
          it_pcl->z = P_compensate(2);

          if (it_pcl == pcl_out.points.begin())
            break;
        }
      }
      std::cout << "undistort_points_num:" << meas.lidar->points.size() << std::endl;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2, t3;
  t1 = omp_get_wtime();

  if (meas.imu.empty())
  {
    return;
  };
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();

  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
