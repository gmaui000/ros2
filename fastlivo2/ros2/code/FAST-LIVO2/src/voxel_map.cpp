/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "voxel_map.h"
using namespace Eigen;
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{
  // 如果载体系点的 z 轴值为 0，则将其置为 0.0001. 避免将 0 作为除数
  if (pb[2] == 0) pb[2] = 0.0001;
  // 计算该点的距离
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  // 计算测距的方差
  float range_var = range_inc * range_inc;
  // 关于方向方差的矩阵
  Eigen::Matrix2d direction_var;
  // 获得方向方差
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
  // 计算该点的方向
  Eigen::Vector3d direction(pb);
  // 对方向归一化（长度变为 1 但方向不变）
  direction.normalize();
  // 计算方向的反对称矩阵
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  // 计算 N 的 N1 和 N2 以及 N
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  // 计算 A
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  // 计算不确定性（协方差）
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void loadVoxelConfig(rclcpp::Node::SharedPtr &node, VoxelMapConfig &voxel_config)
{
  auto try_declare = [node]<typename ParameterT>(const std::string & name,
    const ParameterT & default_value)
  {
    if (!node->has_parameter(name))
    {
      return node->declare_parameter<ParameterT>(name, default_value);
    }
    else
    {
      return node->get_parameter(name).get_value<ParameterT>();
    }
  };

  // declare parameter
  try_declare.template operator()<bool>("publish.pub_plane_en", false);
  try_declare.template operator()<int>("lio.max_layer", 1);
  try_declare.template operator()<double>("lio.voxel_size", 0.5);
  try_declare.template operator()<double>("lio.min_eigen_value", 0.01);
  try_declare.template operator()<double>("lio.sigma_num", 3);
  try_declare.template operator()<double>("lio.beam_err", 0.02);
  try_declare.template operator()<double>("lio.dept_err", 0.05);

  // Declaration of parameter of type std::vector<int> won't build, https://github.com/ros2/rclcpp/issues/1585  
  try_declare.template operator()<vector<int64_t>>("lio.layer_init_num", std::vector<int64_t>{5,5,5,5,5}); 
  try_declare.template operator()<int>("lio.max_points_num", 50);
  try_declare.template operator()<int>("lio.min_iterations", 5);
  try_declare.template operator()<bool>("local_map.map_sliding_en", false);
  try_declare.template operator()<int>("local_map.half_map_size", 100);
  try_declare.template operator()<double>("local_map.sliding_thresh", 8.0);

  // get parameter
  node->get_parameter("publish.pub_plane_en", voxel_config.is_pub_plane_map_);
  node->get_parameter("lio.max_layer", voxel_config.max_layer_);
  node->get_parameter("lio.voxel_size", voxel_config.max_voxel_size_);
  node->get_parameter("lio.min_eigen_value", voxel_config.planner_threshold_);
  node->get_parameter("lio.sigma_num", voxel_config.sigma_num_);
  node->get_parameter("lio.beam_err", voxel_config.beam_err_);
  node->get_parameter("lio.dept_err", voxel_config.dept_err_);
  node->get_parameter("lio.layer_init_num", voxel_config.layer_init_num_);
  node->get_parameter("lio.max_points_num", voxel_config.max_points_num_);
  node->get_parameter("lio.min_iterations", voxel_config.max_iterations_);
  node->get_parameter("local_map.map_sliding_en", voxel_config.map_sliding_en);
  node->get_parameter("local_map.half_map_size", voxel_config.half_map_size);
  node->get_parameter("local_map.sliding_thresh", voxel_config.sliding_thresh);
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
  // 对该节的平面属性进行初始化
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero(); // 平面不确定性
  plane->covariance_ = Eigen::Matrix3d::Zero();            // 协方差
  plane->center_ = Eigen::Vector3d::Zero();                // 中心点
  plane->normal_ = Eigen::Vector3d::Zero();                // 平面法向量
  plane->points_size_ = points.size();                     // 平面的点数
  plane->radius_ = 0;                                      // 最大特征值的开方
  // 遍历该节点的每个不确定性点
  for (auto pv : points)
  {
    // 这里计算的是该节点所有点的累加和，以及世界点与自己本身的转置
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
  }
  // 计算平面协方差 E((P-P")(P-P")^T)=COV P"代表均值（期望）
  plane->center_ = plane->center_ / plane->points_size_;
  plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
  // 获得协方差的特征向量和特征值
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  // 特征值的实数部分
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();
  // 找到 evals 向量的最小值，中间值以及最大值的索引
  Eigen::Matrix3f::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  // 因为特征值向量容器的大小是 3，通过此法可以获得特征值中间值的索引
  int evalsMid = 3 - evalsMin - evalsMax;
  // 获得每个特征值对应的特征向量
  Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
  Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
  Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
  // 生成一个对角阵，元素为点数的倒数
  Eigen::Matrix3d J_Q;
  J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;
  // && evalsReal(evalsMid) > 0.05
  //&& evalsReal(evalsMid) > 0.01
  // 判断最小的特征是否小于平面阈值
  // 获得平面法向量，（基于点不确定性）平面不确定性，平面方程各参数，平面为真标志位，平面更新标志位，平面的 ID 号
  if (evalsReal(evalsMin) < planer_threshold_)
  {
    // 分别求法向量对世界点的导数和中心点对世界点的导数。又已知了世界点的不确定性，因此可以获得平面的不确定性
    for (int i = 0; i < points.size(); i++)
    {
      // 生成一个 63 的矩阵 J Eigen::Matrix<double, 6, 3> J; 和 33 的矩阵 F
      Eigen::Matrix<double, 6, 3> J;
      Eigen::Matrix3d F;
      // 遍历三个特征值索引获得 F
      for (int m = 0; m < 3; m++)
      {
        // 判断 若不是最小值索引 if (m != (int)evalsMin) 则计算该索引处 F的值
        if (m != (int)evalsMin)
        {
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
              (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
          F.row(m) = F_m;
        }
        else
        {
          // 若是最小索引值 F_m 为 0 向量
          Eigen::Matrix<double, 1, 3> F_m;
          F_m << 0, 0, 0;
          F.row(m) = F_m;
        }
      }
      // 获得法向量对点的导数，以及中心点对点的导数
      J.block<3, 3>(0, 0) = evecs.real() * F;
      J.block<3, 3>(3, 0) = J_Q;
      // 获得平面的不确定性
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

    // 更新平面属性
    // 平面法向量
    plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
    // 平面 y 方向的向量（与法向量垂直）
    plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
    // 平面 x 方向的向量（与法向量垂直）
    plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
    // 平面特征值
    plane->min_eigen_value_ = evalsReal(evalsMin);
    plane->mid_eigen_value_ = evalsReal(evalsMid);
    plane->max_eigen_value_ = evalsReal(evalsMax);
    // 平面残差定义为最大特征值的开方
    plane->radius_ = sqrt(evalsReal(evalsMax));
    // 计算平面方程 Ax + By + Cz + d = 0 的 d
    plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) + plane->normal_(2) * plane->center_(2));
    // 设该节点的平面属性为真平面，平面更新标志位设为真
    plane->is_plane_ = true;
    plane->is_update_ = true;
    // 平面是否未初始化 if (!plane->is_init_)，赋予当前平面一个 Id 序号,平面 id 自加，初始化为真
    if (!plane->is_init_)
    {
      plane->id_ = voxel_plane_id;
      voxel_plane_id++;
      plane->is_init_ = true;
    }
  }
  else
  {
    // 否则 最小的特征值大于平面阈值：平面更新设为真，但不是平面
    plane->is_update_ = true;
    plane->is_plane_ = false;
  }
}

void VoxelOctoTree::init_octo_tree()
{
  // 判断该树节点的点数是否超过了点数最小阈值
  if (temp_points_.size() > points_size_threshold_)
  {
    // 调用初始化平面函数
    init_plane(temp_points_, plane_ptr_);
    // 判断是否 为平面
    if (plane_ptr_->is_plane_ == true)
    {
      // 为真该节点就终止了，并将节点中的临时点清空，留出存储
      // 将该树节点设为终止节点
      octo_state_ = 0;
      // new added
      // 判断，若该节点点数是否超出了最大点数
      if (temp_points_.size() > max_points_num_)
      {
        update_enable_ = false;
        std::vector<pointWithVar>().swap(temp_points_);
        new_points_ = 0;
      } // 移除当前节点临时点云，并失能更新标志（此节点不再更新）
    }
    else
    {
      // 否则 该节点状态设为非终止节点继续分割树节点
      octo_state_ = 1;
      cut_octo_tree();
    }
    init_octo_ = true;
    new_points_ = 0;
  }
}

void VoxelOctoTree::cut_octo_tree()
{
  // 如果该节点的层数已经到了最大层，将该节点设为终止点
  if (layer_ >= max_layer_)
  {
    octo_state_ = 0;
    return;
  }
  // 遍历该节点的所有临时点
  // 划分（生成）子叶，计算叶的坐标和中心点坐标，将点推入该叶中。循环后 8 个子节点已全部划分完毕
  for (size_t i = 0; i < temp_points_.size(); i++)
  {
    // 生成一个位置数组(最靠近原点坐标的)
    int xyz[3] = {0, 0, 0};
    // 看似判断实则是计算子叶的坐标
    if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
    if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
    if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
    // 初始化该点所在的叶数
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    // 判断该点所在叶是否为空（是否有八叉树节点（对象））
    if (leaves_[leafnum] == nullptr)
    {
      // 生成一个八叉树节点给该叶，层数在原来父节点层数上加 1
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      // 将点数下限数组给该叶
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      // 计算子叶的中心（在父节点的中心上计算得到）
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      // 为计算下一层的各子叶中心做准备（计算体素边长的八分之一）
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
    }
    // 将这个点推到该子叶的临时点中，子叶的点数更新
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    leaves_[leafnum]->new_points_++;
  }
  // 遍历这八个子叶并只对有效的子叶做处理，一种递归实现
  for (uint i = 0; i < 8; i++)
  {
    if (leaves_[i] != nullptr)
    {
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_)
      {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
        if (leaves_[i]->plane_ptr_->is_plane_)
        {
          leaves_[i]->octo_state_ = 0;
          // new added
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
          {
            leaves_[i]->update_enable_ = false;
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
            new_points_ = 0;
          }
        }
        else
        {
          leaves_[i]->octo_state_ = 1;
          leaves_[i]->cut_octo_tree();
        }
        // 设置初始化节点为真
        leaves_[i]->init_octo_ = true;
        // 设置点数为0
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)
{
  if (!init_octo_)
  {
    // 判断节点是否初始化了（一般是新的节点），将点推入到节点的临时点列表，当点足够时（>5）进行八叉树初始化
    new_points_++;
    temp_points_.push_back(pv);
    if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
  }
  else
  {
    // 否则就是已有节点
    // 若平面属性为真，则根据更新使能与否对平面进行初始化。 
    // 若为假，则去子叶查找（包括生成新叶等操作），再根据更新使能与否 进行平面初始化。
    // 也有可简化的地方（作用有限）：在去子叶查找的那个地方，可以直接判断已有节点的子叶是否更新使能与否，然后在进行八叉树更新。
    
    // 判断 该节点的平面属性是否为平面后再判断是否更新使能（只要点不超过最大点）
    if (plane_ptr_->is_plane_)
    {
      // 推入新点，用于对该节点进行平面初始化。此外，若点数过多，清空临时点
      if (update_enable_)
      {
        // 新点计数累加
        new_points_++;
        // 将不确定性点推入临时点列表
        temp_points_.push_back(pv);
        // 判断 如果点数超过了更新阈值（5），则对该节点进行平面初始化，新点数设为零
        if (new_points_ > update_size_threshold_)
        {
          init_plane(temp_points_, plane_ptr_);
          new_points_ = 0;
        }
        // 判断 临时点点数是否超过最大点数（50），则更新使能设为 false，清空临时点：. 不管这个节点有没有平面，只有点数过多就清空临时点
        if (temp_points_.size() >= max_points_num_)
        {
          update_enable_ = false;
          std::vector<pointWithVar>().swap(temp_points_);
          new_points_ = 0;
        }
      }
    }
    else
    {
      // 否则该节点平面属性不是平面，就去子叶查找

      // 对于非终止叶，可以节点判断后八叉树更新或再分子叶，判断节点继续
      // 八叉树更新。 对于终止叶，直接推入该点，平面初始化，点数超了就
      // 清空临时点，关闭更新使能。
      if (layer_ < max_layer_) // 如果该节点不是最后一层
      {
        // 若是子叶有节点，进行八叉树更新。若是无节点，生成节点，再八叉树更新

        // 计算所在的子叶位置
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        // 判断 该子叶是否有对应节点，有则执行更新八叉树函数
        if (leaves_[leafnum] != nullptr) { leaves_[leafnum]->UpdateOctoTree(pv); }
        else
        {
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
          // 否则就生成新的节点到该子叶再进行八叉树更新
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      }
      else
      {
        // 否则 就是最后一个节点，判断更新使能是否为真
        if (update_enable_)
        {
          // 推入新点，用于对该节点进行平面初始化。此外，若点数过多，清空临时点，update_enable_为 false.（该节点不再进行后续更新）
          new_points_++;
          // 将不确定性点推入临时点列表
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_)
          {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() > max_points_num_)
          {
            update_enable_ = false;
            std::vector<pointWithVar>().swap(temp_points_);
            new_points_ = 0;
          }
        }
      }
    }
  }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

  int xyz[3] = {0, 0, 0};
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  // printf("leafnum: %d. \n", leafnum);

  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv)
{
  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) || (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_)))
  {
    new_points_++;
    temp_points_.push_back(pv);
    return this;
  }

  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_))
  {
    int xyz[3] = {0, 0, 0};
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->Insert(pv); }
    else
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      return leaves_[leafnum]->Insert(pv);
    }
  }
  return nullptr;
}

void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
  // 根据当前帧点云数量初始化反对称阵 cross_mat_list_和载体协方差阵
  // body_cov_list_的列表长度。（用于后面的观测方程雅可比阵）
  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();

  // 遍历当前帧的点，进行不确定性计算和点反对称阵计算。
  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    // 获得当前循环点并避免高度为 0 的点
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (point_this[2] == 0) { point_this[2] = 0.001; }
    M3D var;
    // 计算载体点的不确定性，并将不确定性推入载体系不确定性列表body_cov_list_中
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    body_cov_list_.push_back(var);
    // 将 lidar 系的点转到 IMU 坐标系
    point_this = extR_ * point_this + extT_;
    M3D point_crossmat;
    // 计算 imu 系下点的反对称阵 M3D point_crossmat; 并推入列表cross_mat_list_
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  // 重置 不确定性点列表 pv_list_
  vector<pointWithVar>().swap(pv_list_);
  pv_list_.resize(feats_down_size_);

  // 生成状态系数阵并初始化
  int rematch_num = 0;
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  // 三个关于 EKF 运行情况的标志位：EKF 初始化，EKF 收敛，EKF 停止
  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
  // ESIKF 迭代更新，次数不超过 5 次
  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++)
  {
    // 记录每次迭代的总残差值
    double total_residual = 0.0;
    // 每次迭代得到的世界系雷达点
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    // 旋转协方差 rot_var 和平移协方差 t_var 单拎出来
    M3D rot_var = state_.cov.block<3, 3>(0, 0);
    M3D t_var = state_.cov.block<3, 3>(3, 3);
    // 遍历 lidar 系当前帧点云的每个点
    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
      // 引用重置后的不确定性点（获得世界系和载体系的不确定性）
      pointWithVar &pv = pv_list_[i];
      // 获得载体系和世界系的位置
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

      M3D cov = body_cov_list_[i];
      M3D point_crossmat = cross_mat_list_[i];
      // 计算世界系下点的不确定性
      cov = state_.rot_end * cov * state_.rot_end.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;
      // 将不确定性返回给体素管理器的不确定性点列表 pv
      pv.var = cov;
      pv.body_var = body_cov_list_[i];
    }
    // 清空点到平面相关状态 列表
    ptpl_list_.clear();

    // double t1 = omp_get_wtime();

    // （基于 OpenMP）构建残差列表：点到平面状态
    // 定义于 voxel_map.cpp 将所有匹配到平面的不确定性点的属性及其平面属性 PointToPlane 都推入到 ptpl_list 中
    BuildResidualListOMP(pv_list_, ptpl_list_);

    // build_residual_time += omp_get_wtime() - t1;

    // 累加每个点到平面的距离，做总距离 total_residual
    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    // 获得有效点数量（匹配平面成功的 点数）
    effct_feat_num_ = ptpl_list_.size();
    // 打印计算完残差后的信息
    cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_ 
         << " effective feature num: " << effct_feat_num_ << " average residual: " << total_residual / effct_feat_num_ << endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
    // 关于观测的雅可比阵 H 和测量协方差 R（已经有匹配的点和平面了）
    // 重点是前两行，求得残差，然后根据观测方程对状态误差求导得到 H
    // 阵。 获得每个点对应观测方程的协方差 R（实际上求得是 R 的逆
    // R_inv） 、 雅可比矩阵 Hsub ，以及测量残差（z-h(x,v) = 0 - 点到平
    // 面距离）meas_vec(i) ，便于 K 计算的 H^T * R^-1 Hsub_T_R_inv

    // 初始化一些矩阵用于后续更新 Hsub 测量雅可比矩阵、Hsub_T_R_inv、R_inv 观测、meas_vec
    MatrixXd Hsub(effct_feat_num_, 6);           // 观测方程对误差状态的雅可比
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);
    VectorXd R_inv(effct_feat_num_);
    VectorXd meas_vec(effct_feat_num_);          // 测量向量
    meas_vec.setZero();
    // 遍历所有有效点
    for (int i = 0; i < effct_feat_num_; i++)
    {
      // 计算出每个点对应观测方程的协方差 R（实际上求得是 R 的逆R_inv） 、 雅可比矩阵 Hsub ，以及测量残差（z-h(x,v) = 0 - 点到平
      // 面距离）meas_vec(i) ，便于 K 计算的 H^T * R^-1 Hsub_T_R_inv

      // 计算该有效点的载体点 point_body 和 IMU 系下点 point_this ，并计算 IMU 系点的反对称阵 point_crossmat
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

      // 获得世界系点
      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

      M3D var;
      // V3D normal_b = state_.rot_end.inverse() * ptpl_list_[i].normal_;
      // V3D point_b = ptpl_list_[i].point_b_;
      // double cos_theta = fabs(normal_b.dot(point_b) / point_b.norm());
      // ptpl_list_[i].body_cov_ = ptpl_list_[i].body_cov_ * (1.0 / cos_theta) * (1.0 / cos_theta);

      // point_w cov
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) + (-point_crossmat) * state_propagat.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose();

      // point_w cov (another_version)
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) - point_crossmat * state_propagat.cov.block<3, 3>(0, 0) * point_crossmat;

      // point_body cov
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();

      // 获得点到平面距离协方差中的平面项
      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();

      // 获得 EKF 中观测方程协方差的逆 R_inv(i) 。与标准 EKF 不同，为了避免计算大矩阵的逆，火星实验室使用下式进行计算，与标准 EKF中的增益是相等的
      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H ***/
      // 计算关于旋转误差的雅可比阵
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
      // 计算测量方程的雅可比阵 H Hsub.row(i) 关于平移误差的雅可比阵
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      // 计算计算增益中的 H^T * R^-1 Hsub_T_R_inv
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      // 求观测方程的残差 添加到测量向量中 meas_vec(i)（观测量（0）与估计量（点到平面距离）的差）
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }
    EKF_stop_flg = false;
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
    // 生成一个所有点的增益 K 矩阵（每一个点的 K 构成一个大 K）
    MatrixXd K(DIM_STATE, effct_feat_num_);
    // auto &&Hsub_T = Hsub.transpose();
    // 将 Hsub_T_R_inv 乘以 测量 残差：auto &&HTz = Hsub_T_R_inv * meas_vec; 相当于下式乘以测量残差 z
    auto &&HTz = Hsub_T_R_inv * meas_vec;
    // fout_dbg<<"HTz: "<<HTz<<endl;
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) + state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();
    // 计算协方差更新项 KH
    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
    // 计算传播的先验状态和当前迭代状态的差（盒式减）
    auto vec = state_propagat - state_;
    VD(DIM_STATE)
    // 计算增量
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
    int minRow, minCol;
    // 状态更新
    state_ += solution;
    // 旋转增量 rot_add 和平移增量 t_add
    auto rot_add = solution.block<3, 1>(0, 0);
    auto t_add = solution.block<3, 1>(3, 0);
    // 如果旋转增量的角度小于 0.01 度，或者平移增量的距离小于0.015 厘米，则认为该 EKF 收敛了
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }
    // 记录下当前状态的欧拉角
    V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);

    /*** Rematch Judgement ***/

    // 重匹配判定：只要 EKF 收敛位为真 或 重匹配数为 0 且此时迭代
    // 计数为最大迭代数倒数第二了。重匹配数加 1。 也就是说只有收敛了，
    // 且已经是迭代到最大迭代数的倒数第二了（没到达该数之前都得继续迭
    // 代），rematch_num 才会是 2，后验协方差更新才会满足条件
    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; }

    /*** Convergence Judgements and Covariance Update ***/
    // 收敛判定 以及协方差更新：若 EKF 停止位为假，且 ，达到最大迭代此时
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      // _state.cov = (I_STATE - G) * _state.cov;
      // 更新状态协方差
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      // total_distance += (_state.pos_end - position_last).norm();
      // 最新的位置
      position_last_ = state_.pos_end;
      // 旋转的四元数
      geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
    }
    if (EKF_stop_flg) break;
  }

  // double t2 = omp_get_wtime();
  // scan_count++;
  // ekf_time = t2 - t0 - build_residual_time;

  // ave_build_residual_time = ave_build_residual_time * (scan_count - 1) / scan_count + build_residual_time / scan_count;
  // ave_ekf_time = ave_ekf_time * (scan_count - 1) / scan_count + ekf_time / scan_count;

  // cout << "[ Mapping ] ekf_time: " << ekf_time << "s, build_residual_time: " << build_residual_time << "s" << endl;
  // cout << "[ Mapping ] ave_ekf_time: " << ave_ekf_time << "s, ave_build_residual_time: " << ave_build_residual_time << "s" << endl;
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void VoxelMapManager::BuildVoxelMap()
{
  float voxel_size = config_setting_.max_voxel_size_;                  // 最大体素尺寸
  float planer_threshold = config_setting_.planner_threshold_;         // 平面阈值==0.01
  int max_layer = config_setting_.max_layer_;                          // 从 0 计数，因此总层数为 3 层
  int max_points_num = config_setting_.max_points_num_;                // 最大点数 50 （每个节点）
  std::vector<int> layer_init_num = convertToIntVectorSafe(config_setting_.layer_init_num_);  // 层的初始化点数量阈值

  // 声明一个类型为 不确定性点的向量
  std::vector<pointWithVar> input_points;

  // 遍历世界系下所有的点
  // 根据激光雷达深度误差和轴承误差，计算载体系和世界系点的不确定性，将带不确定性 var
  // 的世界点 point_w 赋予 pointWithVar 对象，并推入到 input_points 中
  for (size_t i = 0; i < feats_down_world_->size(); i++)
  {
    // 声明一个 不确定性点 对象
    pointWithVar pv;
    // 将世界系点坐标给不确定性点的世界系坐标
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
    // 将载体系的点赋予一个三维向量中
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    M3D var;
    // 计算载体系点的不确定性（协方差）（根据深度误差和方向误差）
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    // 进一步计算世界系点的不确定性（协方差）到 pv，并将 pv 推入input_points
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    var = (state_.rot_end * extR_) * var * (state_.rot_end * extR_).transpose() +
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state_.cov.block<3, 3>(3, 3);
    pv.var = var;
    input_points.push_back(pv);
  }

  // 获取 input_points 中的点数
  uint plsize = input_points.size();
  // 遍历所有不确定性点
  for (uint i = 0; i < plsize; i++)
  {
    // 获取当前结构体（点）
    const pointWithVar p_v = input_points[i];
    float loc_xyz[3];
    // 各轴除以体素尺寸获取当前点所在体素的空间位置
    for (int j = 0; j < 3; j++)
    {
      // 比如点[0.8,3.6,2.4]，除以体素尺寸 0.5，可知该点位于：x 轴方向 1.6 个体素，
      // y 轴方向 7.2 个体素，z 轴方向 4.8 个体素处 loc_xyz=[1.6,7.2,4.8] 整
      // 个大体素（方块）坐标：[1,7,4]。至于为什么小于 0 要-1，（这涉及体
      // 素滤波原理）可以这么理解，激光点位置在[0~0.5,0~0.5,0~0.5]范围
      // 内的 整个大体素（方块）坐标为[0,0,0]，那么激光点位置在[0~-
      // 0.5,0~0.5,0~0.5]范围内的整个大体素（方块）坐标为[-1,0,0], 在[0~-
      // 0.5,0~-0.5,0~-0.5]范围内的整个大体素（方块）坐标为[-1,-1,-1]。
      // 以此类推点[-0.8,-3.5,-2.4]的 loc_xyz=[-2.6,-8.2,-5.8]，整个大体素
      // （方块）坐标[-2，-8，-5]
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    // 将 loc_xyz 化为整型后 赋予 体素位置类对象
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    // 利用哈希表返回该体素位置的八叉树索引
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      // 判断 如果不等于哈希表末尾 if (iter != voxel_map_.end())，则说
      // 明该体素方块有八叉树节点，将点推入到该节点中，更新点数
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
    }
    else
    {
      // 否则 说明该体素没有八叉树节点，生成一个根节点
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }
  // 遍历哈希表中的索引对每个根节点进行初始化
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
    iter->second->init_octo_tree();
  }
}

V3F VoxelMapManager::RGBFromVoxel(const V3D &input_point)
{
  int64_t loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
  }

  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
  uint k((ind + 100000) % 3);
  V3F RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
  // cout<<"RGB: "<<RGB.transpose()<<endl;
  return RGB;
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  // 获得该体素图的设置：体素大小，平面阈值，最大层数，最大点数，层初始化点数下限
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = convertToIntVectorSafe(config_setting_.layer_init_num_);
  uint plsize = input_points.size();
  // 遍历每个不确定点
  for (uint i = 0; i < plsize; i++)
  {
    // 引用不确定性点
    const pointWithVar p_v = input_points[i];
    // 计算体素坐标
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    // 查询该体素对应的根节点
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    // 判断，有对应节点，更新该点到八叉树中 （根据新来的点，对节点平面属性进行更新）
    if (iter != voxel_map_.end()) { voxel_map_[position]->UpdateOctoTree(p_v); }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->UpdateOctoTree(p_v);
    }
  }
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  // 获得关于最大层数 max_layer 2，体素尺寸 voxel_size 0.5 和sigma_num 的配置
  int max_layer = config_setting_.max_layer_;
  double voxel_size = config_setting_.max_voxel_size_;
  double sigma_num = config_setting_.sigma_num_;
  std::mutex mylock;
  // 点到平面状态列表清空
  ptpl_list.clear();
  // 声明三个容器：不确定点数来初始化 所有的点到平面状态列表all_ptpl_list（成功的），
  // 有用的点到平面状态 useful_ptpl（点到平面状态是否成功标志容器），索引 index
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
  std::vector<bool> useful_ptpl(pv_list.size());
  std::vector<size_t> index(pv_list.size());
  // 遍历点数给索引列表和有用点到平面列表初始化
  for (size_t i = 0; i < index.size(); ++i)
  {
    index[i] = i;
    useful_ptpl[i] = false;
  }
  // 多线程设置，根据 CPU 核数设置线程数
  #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
  #endif
  // 遍历索引向量
  for (int i = 0; i < index.size(); i++)
  {
    // 引用当前不确定性点
    pointWithVar &pv = pv_list[i];
    // 计算该点的（根节点）体素坐标
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = pv.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    // 根据该位置，利用哈希表找到其对应的八叉树根节点索引
    auto iter = voxel_map_.find(position);
    // 判断 是不是有效索引 获得这一帧所有不确定性点的点到平面状态
    if (iter != voxel_map_.end())
    {
      // 取出索引对应的八叉树节点
      VoxelOctoTree *current_octo = iter->second;
      // 声明一个单个的点到平面状态
      PointToPlane single_ptpl;
      // 声明两个量：点到平面匹配成功标志位 is_sucess；概率 prob
      bool is_sucess = false;
      double prob = 0;
      // 构建单个点到平面残差
      // 计算点到平面的绝对距离，并计算该距离的期望和协方差，若点到平面距离够小则
      // 将匹配成功标志位设为 真 is_sucess = true;，并根据期望和协方差计
      // 算概率 prob，概率值满足后就将点和平面的属性都给 single_ptpl
      build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);
      // 判断 如果 该点到此节点及其子节点的平面都匹配不成功
      if (!is_sucess)
      {
        // 该点在此节点及其子节点的平面都匹配不成功，则移动体素位置到附近体素再计算一次 build_single_residual
        // 计算该点的体素位置的附近体素位置
        VOXEL_LOCATION near_position = position;
        // 以四分之一体素边长为界限，将 loc_xyz 比较 体素中心加上或减去该界限的大小 来移动体素坐标，获得附件体素位置
        if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) { near_position.x = near_position.x + 1; }
        else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) { near_position.x = near_position.x - 1; }
        if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) { near_position.y = near_position.y + 1; }
        else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) { near_position.y = near_position.y - 1; }
        if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) { near_position.z = near_position.z + 1; }
        else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) { near_position.z = near_position.z - 1; }
        // 只要该邻近体素位置有节点，就再计算一次（仅一次）
        auto iter_near = voxel_map_.find(near_position);
        if (iter_near != voxel_map_.end()) { build_single_residual(pv, iter_near->second, 0, is_sucess, prob, single_ptpl); }
      }
      if (is_sucess)
      {
        // 判断，是否匹配到平面。if (is_sucess)。将这一对匹配的点和平面属性存入到 点平面列表中。并设该点为有效点到平面的标志为真
        mylock.lock();
        useful_ptpl[i] = true;
        all_ptpl_list[i] = single_ptpl;
        mylock.unlock();
      }
      else
      {
        // 否则 该点为有效点到平面的标志 为假
        mylock.lock();
        useful_ptpl[i] = false;
        mylock.unlock();
      }
    }
  }
  // 按照有效点平面容器记录的标志，将匹配到平面的不确定性点状态及其到该平面的状态都推入到 ptpl_list 中
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
    if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); }
  }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
  // 初始化最大层数 max_layer 2 和 sigma 数量 3
  int max_layer = config_setting_.max_layer_;
  double sigma_num = config_setting_.sigma_num_;

  // 设置一个残差倍数（用于后续判断）
  double radius_k = 3;
  // 获得不确定性点的世界坐标
  Eigen::Vector3d p_w = pv.point_w;
  // 判断当前节点的平面属性的平面标志位是否为真
  if (current_octo->plane_ptr_->is_plane_)
  {
    // 获得当前节点的平面属性
    VoxelPlane &plane = *current_octo->plane_ptr_;
    // 计算中心点到世界点的向量
    Eigen::Vector3d p_world_to_center = p_w - plane.center_;
    // 根据平面公式计算点到平面距离
    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);
    // 求世界点到平面中心点向量的模长平方
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
    // 世界点到平面中心点向量在平面上投影的模长
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    // 判断这个投影模长是否小于 3 倍的平面协方差的最大特征值
    if (range_dis <= radius_k * plane.radius_)
    {
      // 声明一个 1*6 的雅可比矩阵
      Eigen::Matrix<double, 1, 6> J_nq;
      // 构建这个 J 阵（只构建了前两项）这里将平面不确定性与世界点不确定性分开实现 J。这里关于 J 的代码只将平面不确定性考虑在内
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
      J_nq.block<1, 3>(0, 3) = -plane.normal_;
      // 基于平面不确定性构建协方差第一项和第二项+点不确定性构建第三项=点到平面距离协方差
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
      // 判断 点到平面距离是否小于 3 倍 平方差
      if (dis_to_plane < sigma_num * sqrt(sigma_l))
      {
        is_sucess = true;
        double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        if (this_prob > prob)
        {
          prob = this_prob;
          // 平面与平面上的点共享法向量
          pv.normal = plane.normal_;
          // single_ptpl 相关属性赋值
          // 将匹配点的载体系不确定性 body_cov_、世界系协方差 point_b_ 、世界点 point_w_ 赋予给单个点到平面属性 single_ptpl
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          // 该节点平面不确定性 plane_var_、平面法向量 normal_、平面中心点 center_、平面方程参数 d_、
          // 当前层数 current_layer、点到平面距离 给单个点到平面属性 single_ptpl
          single_ptpl.plane_var_ = plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        }
        return;
      }
      else
      {
        // is_sucess = false;
        return;
      }
    }
    else
    {
      // is_sucess = false;
      return;
    }
  }
  else
  {
    // 否则循环其子节点，并构建单个残差 （递归计算所有子节点的点到平面状态）
    if (current_layer < max_layer)
    {
      for (size_t leafnum = 0; leafnum < 8; leafnum++)
      {
        if (current_octo->leaves_[leafnum] != nullptr)
        {

          VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

void VoxelMapManager::pubVoxelMap()
{
  double max_trace = 0.25;
  double pow_num = 0.2;
  rclcpp::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::msg::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<VoxelPlane> pub_plane_list;
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
    GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) { trace = max_trace; }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane_) { alpha = use_alpha; }
    else { alpha = 0; }
    pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
  }
  voxel_map_pub_->publish(voxel_plane);
  loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
  if (current_octo->layer_ < current_octo->max_layer_)
  {
    if (!current_octo->plane_ptr_->is_plane_)
    {
      for (size_t i = 0; i < 8; i++)
      {
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::msg::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane,
                                     const float alpha, const Eigen::Vector3d rgb)
{
  visualization_msgs::msg::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = rclcpp::Time();
  plane.ns = plane_ns;
  plane.id = single_plane.id_;
  plane.type = visualization_msgs::msg::Marker::CYLINDER;
  plane.action = visualization_msgs::msg::Marker::ADD;
  plane.pose.position.x = single_plane.center_[0];
  plane.pose.position.y = single_plane.center_[1];
  plane.pose.position.z = single_plane.center_[2];
  geometry_msgs::msg::Quaternion q;
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = rclcpp::Duration::from_seconds(0.01);
  plane_pub.markers.push_back(plane);
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::msg::Quaternion &q)
{
  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) { v = vmin; }

  if (v > vmax) { v = vmax; }

  double dr, dg, db;

  if (v < 0.1242)
  {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  }
  else if (v < 0.3747)
  {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  }
  else if (v < 0.6253)
  {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  }
  else if (v < 0.8758)
  {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  }
  else
  {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void VoxelMapManager::mapSliding()
{
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
    std::cout<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position = position_last_;
  double t_sliding_start = omp_get_wtime();
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size, (int64_t)loc_xyz[0] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[1] + config_setting_.half_map_size, (int64_t)loc_xyz[1] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[2] + config_setting_.half_map_size, (int64_t)loc_xyz[2] - config_setting_.half_map_size);
  double t_sliding_end = omp_get_wtime();
  std::cout<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
    const VOXEL_LOCATION& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      // last_delete_time = omp_get_wtime();
      delete it->second;
      it = voxel_map_.erase(it);
      // delete_time += omp_get_wtime() - last_delete_time;
      delete_voxel_cout++;
    } else {
      ++it;
    }
  }
  std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}
