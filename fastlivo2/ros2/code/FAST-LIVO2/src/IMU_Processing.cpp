/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "IMU_Processing.h"
#include <rcpputils/asserts.hpp>

const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

ImuProcess::ImuProcess() : Eye3d(M3D::Identity()),
                           Zero3d(0, 0, 0), b_first_frame(true), imu_need_init(true)
{
  init_iter_num = 1;
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_acc = V3D(0.1, 0.1, 0.1);
  cov_inv_expo = 0.2;
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU = Eye3d;
  last_imu.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  RCLCPP_WARN(rclcpp::get_logger(""), "Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init = true;
  init_iter_num = 1;
  IMUpose.clear();
  last_imu.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::disable_imu()
{
  cout << "IMU Disabled !!!!!" << endl;
  imu_en = false;
  imu_need_init = false;
}

void ImuProcess::disable_gravity_est()
{
  cout << "Online Gravity Estimation Disabled !!!!!" << endl;
  gravity_est_en = false;
}

void ImuProcess::disable_bias_est()
{
  cout << "Bias Estimation Disabled !!!!!" << endl;
  ba_bg_est_en = false;
}

void ImuProcess::disable_exposure_est()
{
  cout << "Online Time Offset Estimation Disabled !!!!!" << endl;
  exposure_estimate_en = false;
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

void ImuProcess::set_gyr_cov_scale(const V3D &scaler) { cov_gyr = scaler; }

void ImuProcess::set_acc_cov_scale(const V3D &scaler) { cov_acc = scaler; }

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_inv_expo_cov(const double &inv_expo) { cov_inv_expo = inv_expo; }

void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcess::set_imu_init_frame_num(const int &num) { MAX_INI_COUNT = num; }

void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  RCLCPP_INFO(rclcpp::get_logger(""),"IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame)
  {
    Reset();
    N = 1;
    b_first_frame = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    // first_lidar_time = meas.lidar_frame_beg_time;
    // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc -
    // mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N); cov_gyr
    // = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr -
    // mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N++;
  }
  IMU_mean_acc_norm = mean_acc.norm();
  state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
  state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  state_inout.bias_g = Zero3d; // mean_gyr;

  last_imu = meas.imu.back();
}

void ImuProcess::Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  pcl_out = *(meas.lidar);
  /*** sort point clouds by offset time ***/
  const double &pcl_beg_time = meas.lidar_frame_beg_time;
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  meas.last_lio_update_time = pcl_end_time;
  const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;

  if (b_first_frame)
  {
    dt = 0.1;
    b_first_frame = false;
  }
  else { dt = pcl_beg_time - time_last_scan; }

  time_last_scan = pcl_beg_time;
  // for (size_t i = 0; i < pcl_out->points.size(); i++) {
  //   if (dt < pcl_out->points[i].curvature) {
  //     dt = pcl_out->points[i].curvature;
  //   }
  // }
  // dt = dt / (double)1000;
  // std::cout << "dt:" << dt << std::endl;
  // double dt = pcl_out->points.back().curvature / double(1000);

  /* covariance propagation */
  // M3D acc_avr_skew;
  M3D Exp_f = Exp(state_inout.bias_g, dt);

  F_x.setIdentity();
  cov_w.setZero();

  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 10) = Eye3d * dt;
  F_x.block<3, 3>(3, 7) = Eye3d * dt;
  // F_x.block<3, 3>(6, 0)  = - R_imu * acc_avr_skew * dt;
  // F_x.block<3, 3>(6, 12) = - R_imu * dt;
  // F_x.block<3, 3>(6, 15) = Eye3d * dt;

  cov_w.block<3, 3>(10, 10).diagonal() = cov_gyr * dt * dt; // for omega in constant model
  cov_w.block<3, 3>(7, 7).diagonal() = cov_acc * dt * dt; // for velocity in constant model
  // cov_w.block<3, 3>(6, 6) =
  //     R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
  // cov_w.block<3, 3>(9, 9).diagonal() =
  //     cov_bias_gyr * dt * dt; // bias gyro covariance
  // cov_w.block<3, 3>(12, 12).diagonal() =
  //     cov_bias_acc * dt * dt; // bias acc covariance

  // std::cout << "before propagete:" << state_inout.cov.diagonal().transpose()
  //           << std::endl;
  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  // std::cout << "cov_w:" << cov_w.diagonal().transpose() << std::endl;
  // std::cout << "after propagete:" << state_inout.cov.diagonal().transpose()
  //           << std::endl;
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

  if (lidar_type != L515)
  {
    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;
    for(; it_pcl != pcl_out.points.begin(); it_pcl--)
    {
        dt_j= pcl_end_offset_time - it_pcl->curvature/double(1000);
        M3D R_jk(Exp(state_inout.bias_g, - dt_j));
        V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
        // Using rotation and translation to un-distort points
        V3D p_jk;
        p_jk = - state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;
  
        V3D P_compensate =  R_jk * P_j + p_jk;
  
        /// save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
    }
  }
}


void ImuProcess::UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  double t0 = omp_get_wtime();
  pcl_out.clear();                                                                  // 清空之前存的去畸变点云
  MeasureGroup &meas = lidar_meas.measures.back();                                  // 取出测量组中最新的Imu队列
  // cout<<"meas.imu.size: "<<meas.imu.size()<<endl;
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu);                                                       // 将上次的最后一个Imu数据推到队列的最前面
  const double &imu_beg_time = stamp2Sec(v_imu.front()->header.stamp);              // 获得Imu队列的前后时间 imu_beg_time， imu_end_time
  const double &imu_end_time = stamp2Sec(v_imu.back()->header.stamp);
  const double prop_beg_time = last_prop_end_time;                                  // 将上次传播的结束时间戳作为本次传播的开始时间戳
  // printf("[ IMU ] undistort input size: %zu \n", lidar_meas.pcl_proc_cur->points.size());
  // printf("[ IMU ] IMU data sequence size: %zu \n", meas.imu.size());
  // printf("[ IMU ] lidar_scan_index_now: %d \n", lidar_meas.lidar_scan_index_now);

  const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;  // 本次传播结束时间为图像获取时间

  /*** cut lidar point based on the propagation-start time and required
   * propagation-end time ***/
  // const double pcl_offset_time = (prop_end_time -
  // lidar_meas.lidar_frame_beg_time) * 1000.; // the offset time w.r.t scan
  // start time auto pcl_it = lidar_meas.pcl_proc_cur->points.begin() +
  // lidar_meas.lidar_scan_index_now; auto pcl_it_end =
  // lidar_meas.lidar->points.end(); printf("[ IMU ] pcl_it->curvature: %lf
  // pcl_offset_time: %lf \n", pcl_it->curvature, pcl_offset_time); while
  // (pcl_it != pcl_it_end && pcl_it->curvature <= pcl_offset_time)
  // {
  //   pcl_wait_proc.push_back(*pcl_it);
  //   pcl_it++;
  //   lidar_meas.lidar_scan_index_now++;
  // }

  // cout<<"pcl_out.size(): "<<pcl_out.size()<<endl;
  // cout<<"pcl_offset_time:  "<<pcl_offset_time<<"pcl_it->curvature:
  // "<<pcl_it->curvature<<endl;
  // cout<<"lidar_meas.lidar_scan_index_now:"<<lidar_meas.lidar_scan_index_now<<endl;

  // printf("[ IMU ] last propagation end time: %lf \n", lidar_meas.last_lio_update_time);
  if (lidar_meas.lio_vio_flg == LIO)                                // 获得待去畸变点云和设置此次传播的第一个IMU的6自由度位姿
  {
    pcl_wait_proc.resize(lidar_meas.pcl_proc_cur->points.size());   // 获得待传播的点云
    pcl_wait_proc = *(lidar_meas.pcl_proc_cur);
    lidar_meas.lidar_scan_index_now = 0;                            // 设激光雷达测量组的当前扫描索引为 0
    // 设置此次传播的第一个IMU位姿 set_pose6d，并推入到IMU位姿队列中 IMUpose
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
  }

  // printf("[ IMU ] pcl_wait_proc size: %zu \n", pcl_wait_proc.points.size());

  // sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // lidar_meas.debug_show();
  // cout<<"UndistortPcl [ IMU ]: Process lidar from "<<prop_beg_time<<" to
  // "<<prop_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to
  //          "<<imu_end_time<<endl;
  // cout<<"[ IMU ]: point size: "<<lidar_meas.lidar->points.size()<<endl;

  /*** Initialize IMU pose ***/
  // IMUpose.clear();

  /*** forward propagation at each imu point ***/
  // 获得上一次前向传播获得的最后时刻 imu 状态以及最新的系统状态
  V3D acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  // cout << "[ IMU ] input state: " << state_inout.vel_end.transpose() << " " << state_inout.pos_end.transpose() << endl;
  M3D R_imu(state_inout.rot_end);           // 获得 IMU 的旋转状态
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;      // 声明状态系数矩阵和过程噪声协方差阵
  double dt, dt_all = 0.0;                  // 声明传播时间间隔和整个传播的时间间隔
  double offs_t;                            // 声明 偏移时间
  // double imu_time;
  double tau;                               // 逆曝光时间
  if (!imu_time_init)
  {
    // imu_time = stamp2Sec(v_imu.front()->header.stamp) - first_lidar_time;
    // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
    tau = 1.0;
    imu_time_init = true;
  }
  else
  {
    tau = state_inout.inv_expo_time;
    // RCLCPP_ERROR_STREAM(rclcpp::get_logger(""),"tau: %.6f !!!!!!", tau);
  }
  // state_inout.cov(6, 6) = 0.01;
  // RCLCPP_ERROR_STREAM(rclcpp::get_logger(""),"lidar_meas.lio_vio_flg");
  // cout<<"lidar_meas.lio_vio_flg: "<<lidar_meas.lio_vio_flg<<endl;
  switch (lidar_meas.lio_vio_flg)
  {
  case LIO:
  case VIO:
    dt = 0;
    // 遍历 imu 队列
    for (int i = 0; i < v_imu.size() - 1; i++)
    {
      auto head = v_imu[i];
      auto tail = v_imu[i + 1];

      // 如果下一个 IMU 的时间 早于 传播开始时间（上次传播的结束时间）--属于上次传播的 IMU 数据。 进行下一次循环
      if (stamp2Sec(tail->header.stamp) < prop_beg_time) continue;

      // 将这个和下个 IMU 数据求平均给 angvel_avr，acc_avr 中值
      angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
          0.5 * (head->angular_velocity.z + tail->angular_velocity.z);

      // angvel_avr<<tail->angular_velocity.x, tail->angular_velocity.y,
      // tail->angular_velocity.z;

      acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x), 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
          0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

      // cout<<"angvel_avr: "<<angvel_avr.transpose()<<endl;
      // cout<<"acc_avr: "<<acc_avr.transpose()<<endl;

      // 将相邻两点的 IMU 到第一帧雷达帧头时间偏移和测量数据均值
      // #ifdef DEBUG_PRINT
      fout_imu << setw(10) << stamp2Sec(head->header.stamp) - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;
      // #endif

      // imu_time = stamp2Sec(head->header.stamp) - first_lidar_time;

      // 将平均角速度减去陀螺仪偏差
      angvel_avr -= state_inout.bias_g;
      // 首先Fast-LIVO需要静止进行初始化（该行代码不适用于运动初始化的恢复）
      // 加速度均值乘以归一化重力值减去加速度偏差
      acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;


      // 判断 当前 IMU 时间是否 早于 上次传播结束时间（上次传播结束时刻卡在当前 imu 和下一个 Imu 之间）
      if (stamp2Sec(head->header.stamp) < prop_beg_time)
      {
        // printf("00 \n");
        dt = stamp2Sec(tail->header.stamp) - last_prop_end_time;
        offs_t = stamp2Sec(tail->header.stamp) - prop_beg_time;
      }
      // 否则 判断 当前 Imu 不是 imu 队列的倒数第二个
      else if (i != v_imu.size() - 2)
      {
        // printf("11 \n");
        dt = stamp2Sec(tail->header.stamp) - stamp2Sec(head->header.stamp);
        offs_t = stamp2Sec(tail->header.stamp) - prop_beg_time;
      }
      // 否则 当前点是倒数第二个 imu
      else
      {
        // printf("22 \n");
        dt = prop_end_time - stamp2Sec(head->header.stamp);
        offs_t = prop_end_time - prop_beg_time;
      }

      // 将时间总间隔加上本次间隔
      dt_all += dt;
      // printf("[ LIO Propagation ] dt: %lf \n", dt);

      /* covariance propagation */
      // 协方差传播
      M3D acc_avr_skew;
      // 根据角速度获得角度增量的指数形式（angvel_avr 包含了偏差）
      M3D Exp_f = Exp(angvel_avr, dt);
      // 获得恢复到 m/s2 单位的加速度反对称阵 用于后续计算
      acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

      // 噪声协方差 Q 阵 13*13
      // 对状态转移矩阵 F_x.setIdentity(); 和系统噪声项cov_w.setZero(); 。维度 19*19，初始化，用于计算 先验协方差
      F_x.setIdentity();
      cov_w.setZero();

      F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
      // 对平移误差状态的导数系数
      if (ba_bg_est_en) F_x.block<3, 3>(0, 10) = -Eye3d * dt;
      // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
      // 对速度误差状态的导数系数
      F_x.block<3, 3>(3, 7) = Eye3d * dt;
      // 对旋转误差状态导数的系数
      F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
      // 对加速度偏差误差导数的系数
      if (ba_bg_est_en) F_x.block<3, 3>(7, 13) = -R_imu * dt;
      // 对重力误差导数的系数
      if (gravity_est_en) F_x.block<3, 3>(7, 16) = Eye3d * dt;

      // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
      // F_x(6,6) = 0.25 * 2 * CV_PI * 0.5 * cos(2 * CV_PI * 0.5 * imu_time) * (-tau*tau); F_x(18,18) = 0.00001;
      // 设置逆曝光协方差项
      if (exposure_estimate_en) cov_w(6, 6) = cov_inv_expo * dt * dt;
      // 旋转误差项对陀螺仪噪声的导数系数与噪声协方差乘。 F_w 的左上角（对微小量做了近似约等于单位阵）
      cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
      // 关于加速度噪声的导数系数与噪声协方差乘
      cov_w.block<3, 3>(7, 7) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
      // 对陀螺仪偏差噪声的导数系数与噪声协方差乘
      cov_w.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt; // bias gyro covariance
      // 对加速度偏差噪声的导数系数与噪声协方差乘
      cov_w.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

      // 状态的先验协方差
      state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
      // state_inout.cov.block<18,18>(0,0) = F_x.block<18,18>(0,0) *
      // state_inout.cov.block<18,18>(0,0) * F_x.block<18,18>(0,0).transpose() +
      // cov_w.block<18,18>(0,0);

      // tau = tau + 0.25 * 2 * CV_PI * 0.5 * cos(2 * CV_PI * 0.5 * imu_time) *
      // (-tau*tau) * dt;

      // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);

      /* propogation of IMU attitude */
      // IMU 的姿态传播
      R_imu = R_imu * Exp_f;

      /* Specific acceleration (global frame) of IMU */
      // IMU 的全局坐标系下加速度
      acc_imu = R_imu * acc_avr + state_inout.gravity;

      /* propogation of IMU */
      // IMU 的位置传播
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

      /* velocity of IMU */
      // IMU 的速度传播
      vel_imu = vel_imu + acc_imu * dt;

      /* save the poses at each IMU measurements */
      // 保存这两个 IMU 数据获得的平均角速度和全局系下加速度
      angvel_last = angvel_avr;
      acc_s_last = acc_imu;

      // cout<<setw(20)<<"offset_t: "<<offs_t<<"stamp2Sec(tail->header.stamp):
      // "<<stamp2Sec(tail->header.stamp)<<endl; printf("[ LIO Propagation ]
      // offs_t: %lf \n", offs_t);
      // 推入 IMU 位姿（包括 下一个 Imu 到开始传播时刻的偏移时间，全局系的 imu 的均值加速度，均值角速度，先验速度，先验位置，先验姿态）
      IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    // unbiased_gyr = V3D(IMUpose.back().gyr[0], IMUpose.back().gyr[1], IMUpose.back().gyr[2]);
    // cout<<"prop end - start: "<<prop_end_time - prop_beg_time<<" dt_all: "<<dt_all<<endl;
    // 将最近的 LIO 更新时刻 更新为 本次 传播结束时刻（图像捕获时刻）
    lidar_meas.last_lio_update_time = prop_end_time;
    // dt = prop_end_time - imu_end_time;
    // printf("[ LIO Propagation ] dt: %lf \n", dt);
    break;
  }

  // 将传播结束时刻的 IMU 速度，IMU 姿态，IMU 位置以及曝光时间更新到状态中。
  state_inout.vel_end = vel_imu;
  state_inout.rot_end = R_imu;
  state_inout.pos_end = pos_imu;
  state_inout.inv_expo_time = tau;

  /*** calculated the pos and attitude prediction at the frame-end ***/
  // if (imu_end_time>prop_beg_time)
  // {
  //   double note = prop_end_time > imu_end_time ? 1.0 : -1.0;
  //   dt = note * (prop_end_time - imu_end_time);
  //   state_inout.vel_end = vel_imu + note * acc_imu * dt;
  //   state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  //   state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 *
  //   acc_imu * dt * dt;
  // }
  // else
  // {
  //   double note = prop_end_time > prop_beg_time ? 1.0 : -1.0;
  //   dt = note * (prop_end_time - prop_beg_time);
  //   state_inout.vel_end = vel_imu + note * acc_imu * dt;
  //   state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  //   state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 *
  //   acc_imu * dt * dt;
  // }

  // cout<<"[ Propagation ] output state: "<<state_inout.vel_end.transpose() <<
  // state_inout.pos_end.transpose()<<endl;

  // 获得队列的最后一个 IMU 消息
  last_imu = v_imu.back();
  // 更新上次传播结束时刻（本次图像捕获时刻）
  last_prop_end_time = prop_end_time;

  double t1 = omp_get_wtime();

  // auto pos_liD_e = state_inout.pos_end + state_inout.rot_end *
  // Lid_offset_to_IMU; auto R_liD_e   = state_inout.rot_end * Lidar_R_to_IMU;

  // cout<<"[ IMU ]: vel "<<state_inout.vel_end.transpose()<<" pos
  // "<<state_inout.pos_end.transpose()<<"
  // ba"<<state_inout.bias_a.transpose()<<" bg
  // "<<state_inout.bias_g.transpose()<<endl; cout<<"propagated cov:
  // "<<state_inout.cov.diagonal().transpose()<<endl;

  //   cout<<"UndistortPcl Time:";
  //   for (auto it = IMUpose.begin(); it != IMUpose.end(); ++it) {
  //     cout<<it->offset_time<<" ";
  //   }
  //   cout<<endl<<"UndistortPcl size:"<<IMUpose.size()<<endl;
  //   cout<<"Undistorted pcl_out.size: "<<pcl_out.size()
  //          <<"lidar_meas.size: "<<lidar_meas.lidar->points.size()<<endl;
  if (pcl_wait_proc.points.size() < 1) return;

  /*** undistort each lidar point (backward propagation), ONLY working for LIO
   * update ***/
  // 开始进行点云去畸变（VIO 模式下不需要）
  if (lidar_meas.lio_vio_flg == LIO)
  {
    // 获得待去畸变点云的最后一个点
    auto it_pcl = pcl_wait_proc.points.end() - 1;
    // 初始化一个旋转 extR_Ri （外参旋转的逆与 tk 时刻全局旋转的逆）和偏移 exrR_extT
    M3D extR_Ri(Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose());
    V3D exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    // 从最后一个 IMU 开始遍历直至队列第一个（倒着）
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      // 上一个 imu 和当前 Imu, 并取出上一个 imu 属性（每个 imu 间隔将从左侧 imu 开始传播）
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      acc_imu << VEC_FROM_ARRAY(head->acc);
      // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      angvel_avr << VEC_FROM_ARRAY(head->gyr);

      // printf("head->offset_time: %lf \n", head->offset_time);
      // printf("it_pcl->curvature: %lf pt dt: %lf \n", it_pcl->curvature,
      // it_pcl->curvature / double(1000) - head->offset_time);

      // 递减遍历：只要这个点的偏移时间 晚于 在左侧 imu 的时间
      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
      {
        // 获得当前点相对左侧 imu 的时间间隔
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame */
        // 获得该点的状态（全局），因为 R_imu 就是全局的
        M3D R_i(R_imu * Exp(angvel_avr, dt));
        // 获得该点到 tk 时刻的平移向量（全局）
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end);

        // 获得该点（相对采样时刻激光雷达系下的点坐标）
        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        // V3D P_compensate = Lid_rot_to_IMU.transpose() *
        // (state_inout.rot_end.transpose() * (R_i * (Lid_rot_to_IMU * P_i +
        // Lid_offset_to_IMU) + T_ei) - Lid_offset_to_IMU);
        // 获得补偿后的点
        V3D P_compensate = (extR_Ri * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) - exrR_extT);

        /// save Undistorted points and their rotation
        // 校正该点--直至到第一个点
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    
    // 将去完畸变的点云复制给 pcl_out，并清空 pcl_wait_proc 和 IMUpose
    pcl_out = pcl_wait_proc;
    pcl_wait_proc.clear();
    IMUpose.clear();
  }
  // printf("[ IMU ] time forward: %lf, backward: %lf.\n", t1 - t0, omp_get_wtime() - t1);
}

void ImuProcess::Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2, t3;
  t1 = omp_get_wtime();
  rcpputils::assert_true(lidar_meas.lidar != nullptr);
  if (!imu_en)
  {
    Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);
    return;
  }

  MeasureGroup meas = lidar_meas.measures.back();

  // 1、记录 pcl 的帧尾时间（相机获取时间）
  // 2、IMU 相关量重置，恢复重力向量单位，设置初始状态的旋转和陀螺仪偏差
  // 3、记录队列最后一个 imu 为最近 last_imu
  // 4、打印 imu 相关量，打开 imu.txt 文件
  // 5、直接返回函数，不进行去畸变（此时为初始化状态，并不开展实际的系统流程，系统实际将在 while (ros::ok()) 第三次循环开始）。
  // 由于未去畸变，feats_undistort 是空的，因此会在后面 LIO 更新时进行返回，进行下一次循环，执行 VIO 更新，
  // 但由于此时啥也没有，也会返回并开始第三次循环，这时候才开始走系统完整流程
  if (imu_need_init)
  {
    double pcl_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;
    // lidar_meas.last_lio_update_time = pcl_end_time;

    if (meas.imu.empty()) { return; };
    /// The very first lidar frame
    IMU_init(meas, stat, init_iter_num);

    imu_need_init = true;

    last_imu = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT)
    {
      // cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init = false;
      RCLCPP_INFO(rclcpp::get_logger(""), "IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: "
               "%.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
               stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm(), cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1],
               cov_gyr[2]);
      RCLCPP_INFO(rclcpp::get_logger(""), "IMU Initials: ba covarience: %.8f %.8f %.8f; bg covarience: "
               "%.8f %.8f %.8f",
               cov_bias_acc[0], cov_bias_acc[1], cov_bias_acc[2], cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }

    return;
  }

  UndistortPcl(lidar_meas, stat, *cur_pcl_un_);
  // cout << "[ IMU ] undistorted point num: " << cur_pcl_un_->size() << endl;
}