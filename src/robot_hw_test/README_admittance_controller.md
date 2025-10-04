# 导纳控制器 (Admittance Controller)

这是一个完整的机械臂导纳控制器实现，支持6DOF笛卡尔空间导纳控制，具备直接使用的能力。

## 功能特性

### 核心功能
- **6DOF笛卡尔空间导纳控制**: 支持位置和姿态的导纳控制
- **实时力/力矩处理**: 完整的六维力输入处理，包括滤波和噪声处理
- **雅可比矩阵计算**: 实现从笛卡尔空间到关节空间的正确转换
- **安全限制**: 包括力限制、位置限制和紧急停止功能
- **实时性能**: 使用实时缓冲区确保控制循环的实时性

### 技术特点
- 基于KDL运动学库的精确运动学计算
- 使用Eigen库进行高效的矩阵运算
- 支持URDF机器人模型
- 可配置的导纳参数
- 低通滤波器减少传感器噪声
- 线程安全的数据传输

## 文件结构

```
robot_hw_test/
├── include/robot_hw_test/
│   └── admittance_controller.h          # 导纳控制器头文件
├── src/
│   └── admittance_controller.cpp        # 导纳控制器实现
├── config/
│   ├── admittance_controller.yaml       # 控制器配置文件
│   └── admittance_controller.rviz       # RViz可视化配置
├── launch/
│   └── admittance_controller.launch     # 启动文件
├── scripts/
│   ├── force_sensor_simulator.py        # 力传感器模拟器
│   └── target_pose_publisher.py         # 目标位姿发布器
└── README_admittance_controller.md      # 本文档
```

## 安装和编译

### 依赖项
确保安装了以下ROS包：
```bash
sudo apt-get install ros-melodic-controller-interface
sudo apt-get install ros-melodic-hardware-interface
sudo apt-get install ros-melodic-kdl-parser
sudo apt-get install ros-melodic-urdf
sudo apt-get install ros-melodic-tf
sudo apt-get install ros-melodic-tf-conversions
sudo apt-get install ros-melodic-realtime-tools
sudo apt-get install ros-melodic-geometry-msgs
sudo apt-get install ros-melodic-sensor-msgs
sudo apt-get install libeigen3-dev
```

### 编译
```bash
cd /home/ljj/fc_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 基本启动
```bash
# 启动导纳控制器
roslaunch robot_hw_test admittance_controller.launch
```

### 2. 配置参数
编辑 `config/admittance_controller.yaml` 文件来调整导纳参数：

```yaml
admittance:
  # 质量矩阵对角线元素 [x, y, z, rx, ry, rz]
  mass_diagonal: [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]
  
  # 阻尼矩阵对角线元素
  damping_diagonal: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
  
  # 刚度矩阵对角线元素
  stiffness_diagonal: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
  
  # 安全限制
  max_force: 50.0      # 最大力 (N)
  max_torque: 10.0     # 最大力矩 (Nm)
  max_velocity: 0.5    # 最大速度 (m/s)
```

### 3. 力输入
控制器订阅以下话题接收力输入：
- `/external_force` (geometry_msgs/WrenchStamped): 外部力/力矩输入

### 4. 目标位姿
控制器订阅以下话题接收目标位姿：
- `/target_pose` (geometry_msgs/PoseStamped): 目标位姿

### 5. 输出话题
控制器发布以下话题：
- `/current_pose` (geometry_msgs/PoseStamped): 当前末端执行器位姿
- `/joint_states` (sensor_msgs/JointState): 关节状态
- `/filtered_wrench` (geometry_msgs/WrenchStamped): 滤波后的力/力矩

## 测试和调试

### 1. 使用力传感器模拟器
```bash
# 启动力传感器模拟器
rosrun robot_hw_test force_sensor_simulator.py

# 设置力模式参数
rosparam set /force_sensor_simulator/force_mode sinusoidal
rosparam set /force_sensor_simulator/force_amplitude 10.0
rosparam set /force_sensor_simulator/force_frequency 0.5
```

### 2. 使用目标位姿发布器
```bash
# 启动目标位姿发布器
rosrun robot_hw_test target_pose_publisher.py

# 设置运动模式
rosparam set /target_pose_publisher/motion_mode circular
rosparam set /target_pose_publisher/motion_amplitude 0.1
```

### 3. 可视化
启动RViz进行可视化：
```bash
rosrun rviz rviz -d src/robot_hw_test/config/admittance_controller.rviz
```

## 导纳控制原理

导纳控制是一种基于力反馈的控制方法，其基本方程为：

```
M * x_desired'' + D * x_desired' + K * x_desired = F_ext
```

其中：
- `M`: 质量矩阵 (6x6)
- `D`: 阻尼矩阵 (6x6)  
- `K`: 刚度矩阵 (6x6)
- `x_desired`: 期望位姿
- `F_ext`: 外部力/力矩

控制器通过以下步骤工作：
1. 接收外部力/力矩输入
2. 应用低通滤波减少噪声
3. 根据导纳方程计算期望加速度
4. 积分得到期望速度和位置
5. 通过雅可比矩阵转换为关节空间
6. 发送关节位置命令

## 参数调优指南

### 质量矩阵 (M)
- 影响系统的惯性响应
- 较大的值使系统响应更慢但更稳定
- 较小的值使系统响应更快但可能不稳定

### 阻尼矩阵 (D)
- 影响系统的阻尼特性
- 较大的值减少振荡但增加响应时间
- 较小的值响应更快但可能产生振荡

### 刚度矩阵 (K)
- 影响系统的刚度
- 较大的值使系统更硬，对外力的抵抗更强
- 较小的值使系统更软，更容易被外力推动

### 滤波参数
- `force_filter_cutoff`: 力滤波截止频率，建议10-50Hz
- `velocity_filter_cutoff`: 速度滤波截止频率，建议5-20Hz

## 安全注意事项

1. **力限制**: 设置合适的最大力和力矩限制
2. **位置限制**: 确保工作空间限制正确设置
3. **速度限制**: 设置合理的最大速度限制
4. **紧急停止**: 确保紧急停止功能正常工作
5. **测试环境**: 在安全环境中进行测试

## 故障排除

### 常见问题

1. **编译错误**
   - 检查所有依赖项是否安装
   - 确保Eigen3库版本兼容

2. **运行时错误**
   - 检查URDF文件路径是否正确
   - 确认关节名称匹配
   - 检查参数配置

3. **控制性能问题**
   - 调整导纳参数
   - 检查滤波参数
   - 确认传感器数据质量

### 调试工具
```bash
# 查看话题列表
rostopic list

# 监控力输入
rostopic echo /external_force

# 监控关节状态
rostopic echo /joint_states

# 查看参数
rosparam list
```

## 扩展功能

### 添加新的力传感器
1. 修改 `externalForceCallback` 函数
2. 添加坐标变换
3. 更新滤波参数

### 添加新的控制模式
1. 扩展 `AdmittanceParams` 结构
2. 修改 `updateAdmittanceControl` 函数
3. 添加相应的参数配置

### 集成真实力传感器
1. 替换力传感器模拟器
2. 添加传感器校准功能
3. 实现传感器数据验证

## 许可证

本项目遵循MIT许可证。

## 贡献

欢迎提交问题报告和功能请求。在提交代码前，请确保：
1. 代码符合项目风格
2. 添加适当的注释
3. 更新相关文档
4. 通过所有测试
