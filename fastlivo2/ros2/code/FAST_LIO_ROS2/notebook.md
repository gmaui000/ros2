# FAST_LIO_ROS2 学习笔记

## 参考文献

[FAST-LIO-ROS2 github](https://github.com/Ericsii/FAST_LIO_ROS2)
[FAST-LIO 公式推导](https://zhuanlan.zhihu.com/p/587500859)
[FAST-LIO notebook](https://blog.csdn.net/Darlingqiang/article/details/142665218)

## 源码的修改

```bash
# 将livox_ros_driver2统一改为livox_ros_driver方便ros2中数据包的转换（ros/ros2/mcap）
grep -ril --exclude='*.md' 'livox_ros_driver2' . | xargs sed -i 's/livox_ros_driver2/livox_ros_driver/gI'
```

## 论文解读

### FAST-LIO 简介
[参考链接](https://blog.csdn.net/wxc_1998/article/details/130909635)

FAST-LIO是港大MaRS实验室在2021年提出的一个紧耦合迭代扩展卡尔曼滤波高计算效率、高鲁棒性的雷达里程计。影响深远，后续又陆续提出了FAST-LIO2以及Faster-LIO等框架。

![FAST-LIO架构图](images/fastlio-arch.png)
![FAST-LIO 符号说明](images/fastlio-symbols.png)

## 技术点深度分析
- [ ] 迭代卡尔曼滤波(IKFOM)原理
- [ ] SO(3)李群数学基础
- [ ] 激光-惯性里程计融合策略
- [ ] ikd-Tree数据结构分析
- [ ] 点云处理算法
- [ ] IMU数据处理流程
- [ ] ROS2系统架构设计

## 源码剖析说明
- [ ] `src/IMU_Processing.hpp` - IMU处理模块
- [ ] `src/preprocess.h/cpp` - 点云预处理
- [ ] `src/laserMapping.cpp` - 激光建图
- [ ] `include/so3_math.h` - SO(3)数学运算
- [ ] `include/use-ikfom.hpp` - IKFOM接口
- [ ] ikd-Tree库集成分析

## 参考文献记录
- [ ] FAST-LIO原始论文
- [ ] 相关SLAM算法文献
- [ ] 卡尔曼滤波理论
- [ ] ROS2官方文档
- [ ] Eigen数学库文档
- [ ] GitHub项目资源

## 实验与实践
- [ ] 环境配置与编译
- [ ] 参数调优指南
- [ ] 性能测试方法
- [ ] 问题排查技巧

## 总结与应用
- [ ] 核心技术总结
- [ ] 算法优势分析
- [ ] 实践应用案例
- [ ] 扩展改进方向

---

*最后更新: 2025-08-25*
*版本: 2.0*
