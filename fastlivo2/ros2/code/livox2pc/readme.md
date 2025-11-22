# `livox2pc`

> Convert livox_ros_driver/CustomMsg point cloud messages from a rosbag into sensor_msgs/PointCloud2, saving results alongside original data into a new rosbag.

## Dependencies

* ROS2 Humble
* [`livox_ros_driver`](https://github.com/Livox-SDK/livox_ros_driver) (in same workspace)
* [`Livox SDK`](https://github.com/Livox-SDK/Livox-SDK2.git) installed

### Usage

```bash
ros2 launch livox2pc livox2std_launch.py rosbag_path:=dataset/Retail_Street

ros2 launch livox2pc proto2livox_launch.py output_format:=std
```

## Reference

```CustomMsg.msg
# Livox publish pointcloud msg format.

std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```

```CustomPoint.msg
# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

```sensor_msgs::msg::PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc.

std_msgs/Header header          # Time of sensor data acquisition, and the coordinate frame ID (for 3D points).
uint32 height                   # 2D structure of the point cloud. 
uint32 width                    # If the cloud is unordered, height is 1 and width is the number of points.
sensor_msgs/PointField[] fields # Describes the channels and their layout in the binary data blob.
bool is_bigendian               # Is this data bigendian?
uint32 point_step               # Length of a point in bytes.
uint32 row_step                 # Length of a row in bytes.
uint8[] data                    # Actual point data, size is (row_step * height).
bool is_dense                   # True if there are no invalid points.
```

```sensor_msgs::msg::PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

# Common PointField names are x, y, z, intensity, rgb, rgba
string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
```
