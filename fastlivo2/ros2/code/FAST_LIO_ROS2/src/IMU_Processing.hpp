#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"

/*
这个hpp主要包含：
IMU数据预处理：IMU初始化，IMU正向传播，反向传播补偿运动失真
*/

//最大迭代次数
#define MAX_INI_COUNT (10)
//判断点的时间先后顺序(注意curvature中存储的是时间戳)
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q; // 噪声协方差矩阵  对应论文式(8)中的Q
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;              // imu参数输出文件 
  V3D cov_acc;                    // 加速度测量协方差
  V3D cov_gyr;                    // 角速度测量协方差
  V3D cov_acc_scale;              // 外部传入的 初始加速度协方差
  V3D cov_gyr_scale;              // 外部传入的 初始角速度协方差
  V3D cov_bias_gyr;               // 角速度bias测量的协方差
  V3D cov_bias_acc;               // 加速度bias测量的协方差
  double first_lidar_time;        // 当前帧第一个点云时间

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;                       // 当前帧点云未去畸变
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;       // 上一帧imu最后的一个值
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;   // imu队列
  vector<Pose6D> IMUpose;                                // 存储imu位姿(反向传播用)
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;                                   // lidar到IMU的旋转外参
  V3D Lidar_T_wrt_IMU;                                   // lidar到IMU的平移外参
  V3D mean_acc;                                          // 加速度均值,用于计算方差
  V3D mean_gyr;                                          // 角速度均值，用于计算方差
  V3D angvel_last;                                       // 上一帧角速度
  V3D acc_s_last;                                        // 上一帧加速度
  double start_timestamp_;                               // 开始时间戳
  double last_lidar_end_time_;                           // 上一帧结束时间戳
  int    init_iter_num = 1;                              // 初始化迭代次数
  bool   b_first_frame_ = true;                          // 是否是第一帧
  bool   imu_need_init_ = true;                          // 是否需要初始化imu
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;                                     // 初始化迭代次数
  Q = process_noise_cov();                               // 调用use-ikfom.hpp里面的process_noise_cov初始化噪声协方差
  cov_acc       = V3D(0.1, 0.1, 0.1);                    // 加速度协方差初始化
  cov_gyr       = V3D(0.1, 0.1, 0.1);                    // 角速度协方差初始化
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);           // 角速度bias协方差初始化
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);           // 加速度bias协方差初始化
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;                              // 上一帧角速度初始化
  Lidar_T_wrt_IMU = Zero3d;                              // lidar到IMU的位置外参初始化
  Lidar_R_wrt_IMU = Eye3d;                               // lidar到IMU的旋转外参初始化
  last_imu_.reset(new sensor_msgs::msg::Imu());          // 上一帧imu初始化
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()  //重置参数
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);                        // 平均角速度/加速度重置
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;                             // 上个角速度量重置
  imu_need_init_    = true;                               // 是否需要初始化imu
  start_timestamp_  = -1;                                 // 开始时间戳
  init_iter_num     = 1;                                  // 初始化迭代次数
  v_imu_.clear();
  IMUpose.clear();                                        // imu位姿清空
  last_imu_.reset(new sensor_msgs::msg::Imu());           // 上一帧imu初始化
  cur_pcl_un_.reset(new PointCloudXYZI());                // 当前帧点云未去畸变初始化
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
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

// IMU初始化：利用开始的IMU帧的平均值初始化状态量x
// 目的：对 IMU 相关量进行重置，恢复重力向量为 m/s2，以及初始化状态旋转阵、陀螺仪偏差及最近Imu 消息等。
void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  // MeasureGroup这个struct表示当前过程中正在处理的所有数据，包含IMU队列和一帧lidar的点云 以及lidar的起始和结束时间
  // 初始化重力、陀螺仪偏差、acc和陀螺仪协方差  将加速度测量值归一化为单位重力   **/
  V3D cur_acc, cur_gyr;                                          // 当前角速度和加速度变量
  
  if (b_first_frame_)                                            // 如果为第一帧IMU
  {
    Reset();                                                     // 对平均加速度/角速度，迭代次数，IMU 位姿容器等进行重置
    N = 1;                                                       // 将迭代次数置1
    b_first_frame_ = false;                                      // 后续非第一帧
    const auto &imu_acc = meas.imu.front()->linear_acceleration; // IMU初始时刻的加速度
    const auto &gyr_acc = meas.imu.front()->angular_velocity;    // IMU初始时刻的角速度
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;                 // 第一帧加速度值作为初始化均值
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;                 // 第一帧角速度值作为初始化均值
    first_lidar_time = meas.lidar_beg_time;                      // 将当前IMU帧对应的lidar起始时间 作为初始时间
  }
  // 计算方差
  for (const auto &imu : meas.imu)                               // 根据所有IMU数据，计算平均值和方差
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;                   // 根据当前帧和均值差作为均值的更新
    mean_gyr      += (cur_gyr - mean_gyr) / N;
    // .cwiseProduct()对应系数相乘
    // 每次迭代之后均值都会发生变化，最后的方差公式中减的应该是最后的均值
    // https://blog.csdn.net/weixin_44479136/article/details/90510374 方差迭代计算公式
    // 按照博客推导出来的下面方差递推公式有两种
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();                     // 在esekfom.hpp获得x_的状态
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);   // 得平均测量的单位方向向量 * 重力加速度预设值
  
  //state_inout.rot = Eye3d;                                     // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;                                     // 角速度测量作为陀螺仪偏差
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;                     // 将lidar到imu外参传入
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);                                 // 将初始化后的状态传入esekfom.hpp中的x_

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();  //在esekfom.hpp获得P_的协方差矩阵
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

// 正向传播 反向传播 去畸变，这里涉及到了Lidar的去畸变问题
void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** 将上一帧最后尾部的imu添加到当前帧头部的imu ***/
  auto v_imu = meas.imu;                                                             // 取出当前帧的IMU队列
  v_imu.push_front(last_imu_);                                                       // 将上一帧最后尾部的imu添加到当前帧头部的imu
  const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();  // 拿到当前帧头部的imu的时间（也就是上一帧尾部的imu时间戳）
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();   // 拿到当前帧尾部的imu的时间
  const double &pcl_beg_time = meas.lidar_beg_time;                                  // 点云开始和结束的时间戳
  const double &pcl_end_time = meas.lidar_end_time;
  
  // 根据点云中每个点的时间戳对点云进行重排序
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list); // 这里curvature中存放了时间戳（在preprocess.cpp中）
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  // 获取上一次KF估计的后验状态作为本次IMU预测的初始状态
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  // 将初始状态加入IMUpose中,包含有时间间隔，上一帧加速度，上一帧角速度，上一帧速度，上一帧位置，上一帧旋转矩阵

  // angvel_avr为平均角速度，acc_avr为平均加速度，acc_imu为imu加速度，vel_imu为imu速度，pos_imu为imu位置
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu; // IMU旋转矩阵 消除运动失真的时候用

  double dt = 0;

  input_ikfom in;
  // 遍历本次估计的所有IMU测量并且进行积分，离散中值法 前向传播
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);      // 拿到当前帧的imu数据
    auto &&tail = *(it_imu + 1);  // 拿到下一帧的imu数据

    double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
    double head_stamp = rclcpp::Time(head->header.stamp).seconds();

    // 判断时间先后顺序：下一帧时间戳是否小于上一帧结束时间戳 不符合直接continue
    if (tail_stamp < last_lidar_end_time_)    continue;
    
    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),        // 中值积分
                  0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                  0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr    << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                  0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                  0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;  // 通过重力数值对加速度进行调整(除上初始化的IMU大小*9.8)

    // 如果IMU开始时刻早于上次雷达最晚时刻(因为将上次最后一个IMU插入到此次开头了，所以会出现一次这种情况)
    if(head_stamp < last_lidar_end_time_)
    {
      dt = tail_stamp - last_lidar_end_time_;         // 从上次雷达时刻末尾开始传播 计算与此次IMU结尾之间的时间差
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail_stamp - head_stamp;                   // 两个IMU时刻之间的时间间隔
    }
    
    in.acc = acc_avr;                                 // 两帧IMU的中值作为输入in  用于前向传播
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;         // 配置协方差矩阵
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);                      // IMU前向传播，每次传播的时间间隔为dt

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();                     // 更新IMU状态为积分后的状态
    angvel_last = angvel_avr - imu_state.bg;          // 更新上一帧角速度 = 后一帧角速度-bias
    // 更新上一帧世界坐标系下的加速度 = R*(加速度-bias) - g
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail_stamp - pcl_beg_time;      // 后一个IMU时刻距离此次雷达开始的时间间隔
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  // 把最后一帧IMU测量也补上
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();                         // 保存最后一个IMU测量，以便于下一帧使用
  last_lidar_end_time_ = pcl_end_time;                 // 保存这一帧最后一个雷达测量的结束时间，以便于下一帧使用

  /*** undistort each lidar point (backward propagation) ***/
  /*** 消除每个激光雷达点的失真（反向传播）***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  // 遍历每个IMU帧
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);                   // 拿到前一帧的IMU旋转矩阵
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);                 // 拿到前一帧的IMU速度
    pos_imu<<VEC_FROM_ARRAY(head->pos);                 // 拿到前一帧的IMU位置
    acc_imu<<VEC_FROM_ARRAY(tail->acc);                 // 拿到后一帧的IMU加速度
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);              // 拿到后一帧的IMU角速度
  
    // 之前点云按照时间从小到大排序过，IMUpose也同样是按照时间从小到大push进入的
    // 此时从IMUpose的末尾开始循环，也就是从时间最大处开始，因此只需要判断 点云时间需>IMU head时刻  即可   不需要判断 点云时间<IMU tail
    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time; // 点到IMU开始时刻的时间间隔 

      /*变换到“结束”帧，仅使用旋转
       *注意：补偿方向与帧的移动方向相反
       *所以如果我们想补偿时间戳i到帧e的一个点
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei)  其中T_ei在全局框架中表示*/
      M3D R_i(R_imu * Exp(angvel_avr, dt));                      // 点it_pcl所在时刻的旋转：前一帧的IMU旋转矩阵 * exp(后一帧角速度*dt)
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);                  // 点所在时刻的位置(雷达坐标系下)
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos); // 从点所在的世界位置-雷达末尾世界位置
      //.conjugate()取旋转矩阵的共轭,rot.conjugate（）是四元数共轭，即旋转求逆
      // imu_state.offset_R_L_I是从雷达到惯性的旋转矩阵 简单记为I^R_L
      // imu_state.offset_T_L_I是惯性系下雷达坐标系原点的位置简单记为I^t_L
      //下面去畸变补偿的公式这里倒推一下
      // e代表end时刻
      // P_compensate是点在末尾时刻在雷达系的坐标 简记为L^P_e
      //将右侧矩阵乘过来并加上右侧平移
      //左边变为I^R_L * L^P_e + I^t_L= I^P_e 也就是end时刻点在IMU系下的坐标
      //右边剩下imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei
      // imu_state.rot.conjugate()是结束时刻IMU到世界坐标系的旋转矩阵的逆函数 也就是(W^R_i_e)^T
      // T_ei展开是pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos也就是点所在时刻IMU在世界坐标系下的位置 - end时刻IMU在世界坐标系下的位置 W^t_I-W^t_I_e
      //现在等式两边变为 I^P_e =  (W^R_i_e)^T * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + W^t_I - W^t_I_e
      //(W^R_i_e) * I^P_e + W^t_I_e = (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + W^t_I
      // 世界坐标系也无所谓时刻了 因为只有一个世界坐标系 两边变为
      // W^P = R_i * I^P+ W^t_I
      // W^P = W^P
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);         // 如果开头几帧  需要初始化IMU参数

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);   // 在上面IMU_init()基础上乘上缩放系数
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      std::cout << "IMU Initial Done" << std::endl;
      printf("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
                imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);

      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  //正向传播 反向传播 去畸变
  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
