#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver/msg/custom_msg.hpp>
#include <livox_ros_driver/msg/custom_point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;
using sensor_msgs::msg::PointField;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;

sensor_msgs::msg::PointCloud2 livox2pc(const livox_ros_driver::msg::CustomMsg &livox_msg)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = livox_msg.header.frame_id;
    cloud_msg.header.stamp = livox_msg.header.stamp;
    cloud_msg.height = 1;
    cloud_msg.width = livox_msg.points.size();
    // cloud_msg.fields.resize(7);
    // 32位是4个字节
    cloud_msg.fields.resize(5);
    cloud_msg.fields[1].offset = 0;
    cloud_msg.fields[1].name = "x";
    cloud_msg.fields[1].count = 1;
    cloud_msg.fields[1].datatype = PointField::FLOAT32;
    cloud_msg.fields[2].offset = 4;
    cloud_msg.fields[2].name = "y";
    cloud_msg.fields[2].count = 1;
    cloud_msg.fields[2].datatype = PointField::FLOAT32;
    cloud_msg.fields[3].offset = 8;
    cloud_msg.fields[3].name = "z";
    cloud_msg.fields[3].count = 1;
    cloud_msg.fields[3].datatype = PointField::FLOAT32;
    cloud_msg.fields[4].offset = 12;
    cloud_msg.fields[4].name = "intensity";
    cloud_msg.fields[4].count = 1;
    cloud_msg.fields[4].datatype = PointField::FLOAT32;
    cloud_msg.fields[0].offset = 16;
    cloud_msg.fields[0].name = "offset_time";
    cloud_msg.fields[0].count = 1;
    cloud_msg.fields[0].datatype = PointField::UINT32;
    // cloud_msg.fields[5].offset = 20;
    // cloud_msg.fields[5].name = "tag";
    // cloud_msg.fields[5].count = 1;
    // cloud_msg.fields[5].datatype = PointField::UINT8;
    // cloud_msg.fields[6].offset = 21;
    // cloud_msg.fields[6].name = "line";
    // cloud_msg.fields[6].count = 1;
    // cloud_msg.fields[6].datatype = PointField::UINT8;
    cloud_msg.point_step = sizeof(livox_ros_driver::msg::CustomPoint);
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(cloud_msg.row_step);

    sensor_msgs::PointCloud2Iterator<uint32_t> iter_offset_time(cloud_msg, "offset_time");
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(cloud_msg, "tag");
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_line(cloud_msg, "line");

    for (const auto &p : livox_msg.points)
    {
        *iter_offset_time = p.offset_time;
        *iter_x = p.x;
        *iter_y = p.y;
        *iter_z = p.z;
        *iter_intensity = p.reflectivity;
        // *iter_tag = p.tag;
        // *iter_line = p.line;

        ++iter_offset_time;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
        // ++iter_tag;
        // ++iter_line;
    }

    return cloud_msg;
}

livox_ros_driver::msg::CustomMsg pc2livox(const sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    livox_ros_driver::msg::CustomMsg livox_msg;
    livox_msg.header = cloud_msg.header;
    livox_msg.points.resize(cloud_msg.width);

    sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_offset_time(cloud_msg, "offset_time");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(cloud_msg, "intensity");

    for (auto &livox_p : livox_msg.points) {
        livox_p.offset_time = *iter_offset_time;
        livox_p.x = *iter_x;
        livox_p.y = *iter_y;
        livox_p.z = *iter_z;
        livox_p.reflectivity = *iter_intensity;
        livox_p.tag = 0;
        livox_p.line = 0;

        ++iter_offset_time;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
    }

    return livox_msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("livox2pc_node");

    std::string livox_lidar_topic = node->declare_parameter<std::string>("livox_lidar_topic", "/livox/lidar");
    std::string rosbag_path = node->declare_parameter<std::string>("rosbag_path", "");
    std::string livox2pc_lidar_topic = node->declare_parameter<std::string>("livox2pc_lidar_topic", "/livox/points");

    // 在线模式：如果没有指定rosbag_path，则订阅在线话题并发布转换后的消息
    if (rosbag_path.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Running in online mode, subscribing to topic: %s", livox_lidar_topic.c_str());
        RCLCPP_INFO(node->get_logger(), "Publishing converted pointcloud to topic: %s", livox2pc_lidar_topic.c_str());

        // 创建发布器
        auto livox2pc_lidar_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(livox2pc_lidar_topic, 10);

        // 创建订阅器回调函数
        auto livox_lidar_callback = [livox2pc_lidar_pub](const livox_ros_driver::msg::CustomMsg::SharedPtr livox_msg) {
            auto livox2pc_msg = livox2pc(*livox_msg);
            livox2pc_lidar_pub->publish(livox2pc_msg);
        };

        // 创建订阅器
        auto livox_lidar_sub = node->create_subscription<livox_ros_driver::msg::CustomMsg>(
            livox_lidar_topic, 10, livox_lidar_callback);

        RCLCPP_INFO(node->get_logger(), "Online conversion started. Press Ctrl+C to exit.");
        rclcpp::spin(node);
    } else {
        rosbag2_cpp::Reader reader;
        try
        {
            reader.open(rosbag_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "loading bag failed: %s", e.what());
            return -1;
        }

        std::string out_bag_path = rosbag_path.substr(0, rosbag_path.find_last_of(".")) + "_converted";
        rosbag2_cpp::Writer writer;
        try
        {
            writer.open(out_bag_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "failed to open output bag: %s", e.what());
            return -1;
        }

        // 遍历所有消息
        size_t count = 0;
        // Get all topics from input bag and create them in output bag
        auto topics = reader.get_all_topics_and_types();
        for (const auto& topic : topics) {
            try {
                writer.create_topic(topic);
            } catch (const std::exception& e) {
                RCLCPP_WARN(node->get_logger(), "Failed to create topic %s: %s", 
                        topic.name.c_str(), e.what());
            }
        }

        // Create our custom pointcloud topic
        rosbag2_storage::TopicMetadata pointcloud_topic;
        pointcloud_topic.name = "/livox/pointcloud2";
        pointcloud_topic.type = "sensor_msgs/msg/PointCloud2";
        pointcloud_topic.serialization_format = "cdr";
        writer.create_topic(pointcloud_topic);

        while (reader.has_next())
        {
            auto serialized_msg = reader.read_next();
            count++;
            
            // 显示进度（每100条显示一次）
            if (count % 100 == 0)
            {
                RCLCPP_INFO(node->get_logger(), "Processing message %zu", count);
            }

            // 转换点云消息类型
            if (serialized_msg->topic_name == livox_lidar_topic)
            {
                auto livox_msg = std::make_shared<livox_ros_driver::msg::CustomMsg>();
                rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);
                rclcpp::Serialization<livox_ros_driver::msg::CustomMsg> serializer;
                serializer.deserialize_message(&extracted_serialized_msg, livox_msg.get());

                auto pc2_msg = livox2pc(*livox_msg);
                auto serialized_pc2 = std::make_shared<rclcpp::SerializedMessage>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;
                pc2_serializer.serialize_message(&pc2_msg, serialized_pc2.get());
                writer.write(serialized_pc2, "/livox/pointcloud2", "sensor_msgs/msg/PointCloud2",
                            rclcpp::Time(serialized_msg->time_stamp));
            }
            else
            {
                writer.write(serialized_msg);
            }
        }

        RCLCPP_INFO(node->get_logger(), "Finished writing converted bag: %s", out_bag_path.c_str());
    }
    rclcpp::shutdown();
    return 0;
}
