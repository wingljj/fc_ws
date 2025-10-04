#!/bin/bash

# 导纳控制器编译和测试脚本

echo "=== 导纳控制器编译和测试脚本 ==="

# 设置工作空间
cd /home/ljj/fc_ws

echo "1. 清理之前的编译..."
catkin_make clean

echo "2. 编译工作空间..."
catkin_make

if [ $? -eq 0 ]; then
    echo "✓ 编译成功"
else
    echo "✗ 编译失败"
    exit 1
fi

echo "3. 设置环境..."
source devel/setup.bash

echo "4. 检查ROS包..."
rospack find robot_hw_test

echo "5. 检查控制器插件..."
rospack plugins --attrib=plugin controller_interface | grep admittance

echo "6. 启动测试环境..."
echo "请在新终端中运行以下命令进行测试："
echo ""
echo "# 启动导纳控制器"
echo "roslaunch robot_hw_test admittance_controller.launch"
echo ""
echo "# 在另一个终端中运行测试脚本"
echo "rosrun robot_hw_test test_admittance_controller.py"
echo ""
echo "# 或者手动测试力传感器模拟器"
echo "rosrun robot_hw_test force_sensor_simulator.py"
echo ""
echo "# 手动测试目标位姿发布器"
echo "rosrun robot_hw_test target_pose_publisher.py"
echo ""
echo "=== 编译和测试脚本完成 ==="
