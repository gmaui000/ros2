// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver/msg/custom_msg.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// 枚举类型：表示支持的雷达类型
enum LID_TYPE
{
  AVIA = 1,
  VELO16,
  OUST64,
  MID360,
  HELIOS,
};

// 枚举类型：表征雷达时间偏移单位
enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

// 枚举类型：表示特征点的类型
enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};

// 枚举类型：位置标识
enum Surround
{
  Prev,
  Next
};

// 枚举类型：表示有跨越边的类型
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

// orgtype类：用于存储激光雷达点的一些其他属性
struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor; //默认为正常点
    intersect = 2;
  }
};

// velodyne数据结构
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// 注册velodyne_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(float, time, time)(uint16_t, ring,
                                                                                                        ring))

/*** Helios ***/
namespace helios_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  int8_t ring;
  int32_t time_offset;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace helios_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(helios_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(int8_t, ring, ring)(int32_t, time_offset, time_offset))
/****************/

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

namespace livox_ros
{
typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
} LivoxPointXyzrtl;

typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float intensity;    /**< Intensity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
} LivoxPointXyzitl;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzrtl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzitl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;                           // 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128];                                        //maximum 128 line lidar
  vector<orgtype> typess[128];                                        //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;    // 雷达类型、采样间隔、扫描线数、扫描频率、时间单位
  double blind, blind_sqr;                                            // 最小距离阈值(盲区)
  bool feature_enabled, given_offset_time;                            // 是否提取特征、是否进行时间偏移
  // ros::Publisher pub_full, pub_surf, pub_corn;                     // 发布全部点、发布平面点、发布边缘点

private:
  void avia_handler(const livox_ros_driver::msg::CustomMsg::UniquePtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void helios_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
