cmake_minimum_required(VERSION 3.8)
project(livox2pc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Protobuf REQUIRED)

include(FindProtobuf)


# ---------------------------
# Proto 输出目录
# ---------------------------
set(PROTO_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

# ---------------------------
# 查找所有 proto 文件
# ---------------------------
file(GLOB_RECURSE PROTO_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/protocol/foxglove/*.proto"
    "${CMAKE_CURRENT_SOURCE_DIR}/protocol/protobuf/*.proto"
    "${CMAKE_CURRENT_SOURCE_DIR}/protocol/primitive/sensor/*.proto"
)

# ---------------------------
# 自动生成 C++ 源码，保持目录结构
# ---------------------------
set(PROTO_SRCS)
set(PROTO_HDRS)

foreach(PROTO_FILE ${PROTO_FILES})
    # 计算 proto 文件相对于 proto 根目录的相对路径
    # 假设 proto 根目录是 protocol/
    file(RELATIVE_PATH REL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/protocol ${PROTO_FILE})
    get_filename_component(REL_DIR ${REL_PATH} DIRECTORY)
    get_filename_component(BASE_NAME ${PROTO_FILE} NAME_WE)

    # 创建对应的输出子目录
    set(OUT_DIR ${PROTO_OUT_DIR}/${REL_DIR})
    file(MAKE_DIRECTORY ${OUT_DIR})

    # 输出的 pb 文件路径
    set(OUT_CPP ${OUT_DIR}/${BASE_NAME}.pb.cc)
    set(OUT_H   ${OUT_DIR}/${BASE_NAME}.pb.h)

    add_custom_command(
        OUTPUT ${OUT_CPP} ${OUT_H}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
            -I${CMAKE_CURRENT_SOURCE_DIR}/protocol
            -I${CMAKE_CURRENT_SOURCE_DIR}/protocol/primitive/sensor
            --cpp_out=${PROTO_OUT_DIR}
            ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating C++ from ${PROTO_FILE}"
        VERBATIM
    )

    list(APPEND PROTO_SRCS ${OUT_CPP})
    list(APPEND PROTO_HDRS ${OUT_H})
endforeach()

# 生成一个静态库 target，方便 link
add_library(proto_lib ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(proto_lib PUBLIC ${PROTO_OUT_DIR})
target_link_libraries(proto_lib PUBLIC ${Protobuf_LIBRARIES})

# proto2livox
add_executable(proto2livox src/proto2livox.cpp ${PROTO_SRCS})
target_link_libraries(proto2livox proto_lib)
ament_target_dependencies(proto2livox
  rclcpp
  sensor_msgs
  livox_ros_driver
)
target_compile_options(proto2livox PRIVATE -Wall)

# livox2std
add_executable(livox2std src/livox2std.cpp)
ament_target_dependencies(livox2std
  rclcpp
  sensor_msgs
  rosbag2_cpp
  livox_ros_driver
)
target_compile_options(livox2std PRIVATE -Wall)

install(TARGETS livox2std proto2livox
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
