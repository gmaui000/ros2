#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "primitive/sensor/sensor_msgs.pb.h"
#include "livox_ros_driver/msg/custom_msg.hpp"
#include "foxglove/PointCloud.pb.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <variant>

using namespace std::chrono_literals;
using sensor_msgs::msg::PointField;

class Proto2LivoxConverter : public rclcpp::Node {
public:
  Proto2LivoxConverter() : Node("proto2livox_converter") {
    // Declare output format parameter
    this->declare_parameter<std::string>("output_format", "livox");
    output_format_ = this->get_parameter("output_format").as_string();
    
    if (output_format_ != "livox" && output_format_ != "std") {
      RCLCPP_ERROR(this->get_logger(), "Invalid output_format: %s, must be 'livox' or 'std'", 
                  output_format_.c_str());
      throw std::runtime_error("Invalid output_format parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Output format set to: %s (valid options: livox, std)", 
               output_format_.c_str());
    // Create subscriber for raw protobuf data
    proto_imu_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/imu", 10,
      [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        asp_sensor::Imu proto_msg;
        if (proto_msg.ParseFromArray(msg->data.data(), msg->data.size())) {
          auto livox_imu = convertToLivoxImu(proto_msg);
          livox_imu_pub_->publish(livox_imu);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf IMU message");
        }
      });
      
    // Initialize QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    // Create publishers
    livox_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", qos);
    
    if (output_format_ == "livox") {
      livox_pub_ = this->create_publisher<livox_ros_driver::msg::CustomMsg>("/livox/lidar", qos);
    } else {
      std_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2", qos);
    }
    
    // Create LiDAR subscriber
    proto_lidar_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/main/ruby/lidar_points", qos,
      [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        foxglove::PointCloud proto_msg;
        if (proto_msg.ParseFromArray(msg->data.data(), msg->data.size())) {
          if (output_format_ == "livox") {
            auto livox_lidar = convertToLivoxLidar(proto_msg);
            livox_pub_->publish(livox_lidar);
          } else {
            auto std_lidar = convertToStandardLidar(proto_msg);
            std_pub_->publish(std_lidar);
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf IMU message");
        }
      });
  }

private:
  sensor_msgs::msg::Imu convertToLivoxImu(const asp_sensor::Imu& proto_msg) {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = rclcpp::Time(proto_msg.header().stamp().sec(), proto_msg.header().stamp().nanosec());
    msg.header.frame_id = "proto_imu";

    // Convert orientation
    const auto& ori = proto_msg.orientation();
    msg.orientation.x = ori.x();
    msg.orientation.y = ori.y();
    msg.orientation.z = ori.z();
    msg.orientation.w = ori.w();

    // Convert angular velocity
    const auto& ang = proto_msg.angular();
    msg.angular_velocity.x = ang.x();
    msg.angular_velocity.y = ang.y();
    msg.angular_velocity.z = ang.z();

    // Convert linear acceleration
    const auto& acc = proto_msg.acceleration();
    msg.linear_acceleration.x = acc.x();
    msg.linear_acceleration.y = acc.y();
    msg.linear_acceleration.z = acc.z();

    // Helper lambda for covariance conversion
    auto convert_covariance = [](auto& dest, const auto& src) {
      if (src.size() >= 9) {
        std::copy_n(src.begin(), 9, dest.begin());
      }
    };

    // Convert covariance matrices
    convert_covariance(msg.orientation_covariance, proto_msg.orientation_covariance());
    convert_covariance(msg.angular_velocity_covariance, proto_msg.angular_covariance());
    convert_covariance(msg.linear_acceleration_covariance, proto_msg.acceleration_covariance());

    return msg;
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr proto_imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr livox_imu_pub_;

  livox_ros_driver::msg::CustomMsg convertToLivoxLidar(const foxglove::PointCloud& proto_msg) {
    auto msg = livox_ros_driver::msg::CustomMsg();
    msg.header.stamp = rclcpp::Time(proto_msg.timestamp().seconds(), proto_msg.timestamp().nanos());
    msg.header.frame_id = proto_msg.frame_id();

    const auto& fields = proto_msg.fields();
    const auto& data = proto_msg.data();
    const size_t point_count = data.size() / proto_msg.point_stride();
    msg.point_num = point_count;
    msg.points.resize(point_count);

    // Create field offset map
    std::unordered_map<std::string, int> field_offsets;
    for (const auto& field : fields) {
      field_offsets[field.name()] = field.offset();
    }

    // Check required fields
    if (!field_offsets.count("x") || !field_offsets.count("y") || !field_offsets.count("z")) {
      RCLCPP_ERROR(this->get_logger(), "Missing required fields (x,y,z) in point cloud");
      return msg;
    }

    // Convert points
    const uint8_t* raw_data = reinterpret_cast<const uint8_t*>(data.data());
    const bool has_intensity = field_offsets.count("intensity");

    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t* point_data = raw_data + i * proto_msg.point_stride();
      auto& point = msg.points[i];

      point.x = *reinterpret_cast<const float*>(point_data + field_offsets["x"]);
      point.y = *reinterpret_cast<const float*>(point_data + field_offsets["y"]);
      point.z = *reinterpret_cast<const float*>(point_data + field_offsets["z"]);
      point.reflectivity = has_intensity ? 
        *reinterpret_cast<const float*>(point_data + field_offsets["intensity"]) : 0.0f;
      point.offset_time = 0;
      point.tag = 0;
      point.line = 0;
    }

    if (proto_msg.has_pose()) {
      RCLCPP_WARN(this->get_logger(), "Pose transformation not yet implemented");
    }

    return msg;
  }

  sensor_msgs::msg::PointCloud2 convertToStandardLidar(const foxglove::PointCloud& proto_msg) {
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.stamp = rclcpp::Time(proto_msg.timestamp().seconds(), proto_msg.timestamp().nanos());
    // msg.header.frame_id = proto_msg.frame_id();
    msg.header.frame_id = "proto_frame";
    msg.height = 1;
    msg.width = proto_msg.data().size() / proto_msg.point_stride();

    // Convert fields from proto to PointCloud2
    msg.fields.clear();
    for (const auto& proto_field : proto_msg.fields()) {
      sensor_msgs::msg::PointField field;
      field.name = proto_field.name();
      field.offset = proto_field.offset();
      field.datatype = proto_field.type();    
      field.count = 1;
      msg.fields.push_back(field);
    }

    msg.point_step = proto_msg.point_stride();
    msg.row_step = msg.width * msg.point_step;
    const auto& proto_data = proto_msg.data();
    msg.data.assign(proto_data.begin(), proto_data.end());

    return msg;
  }

  std::string output_format_;
  rclcpp::Publisher<livox_ros_driver::msg::CustomMsg>::SharedPtr livox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr std_pub_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr proto_lidar_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Proto2LivoxConverter>());
  rclcpp::shutdown();
  return 0;
}
