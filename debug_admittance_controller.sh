#!/bin/bash

# 导纳控制器调试脚本

echo "=== 导纳控制器调试脚本 ==="

# 设置环境
source /home/ljj/fc_ws/devel/setup.bash

echo "1. 检查ROS话题..."
echo "当前活跃的话题："
rostopic list | grep -E "(external_force|current_pose|joint_states|filtered_wrench)"

echo ""
echo "2. 检查控制器状态..."
echo "当前运行的控制器："
rosservice call /controller_manager/list_controllers

echo ""
echo "3. 检查参数..."
echo "导纳控制器参数："
rosparam get /admittance_controller/admittance 2>/dev/null || echo "参数未找到"

echo ""
echo "4. 监控力输入话题..."
echo "监控 /external_force 话题（按Ctrl+C停止）："
timeout 5s rostopic echo /external_force --noarr || echo "没有检测到力输入"

echo ""
echo "5. 监控关节状态..."
echo "监控 /joint_states 话题（按Ctrl+C停止）："
timeout 5s rostopic echo /joint_states --noarr || echo "没有检测到关节状态"

echo ""
echo "6. 监控当前位姿..."
echo "监控 /current_pose 话题（按Ctrl+C停止）："
timeout 5s rostopic echo /current_pose --noarr || echo "没有检测到位姿信息"

echo ""
echo "=== 调试建议 ==="
echo "1. 确保导纳控制器正在运行："
echo "   roslaunch robot_hw_test admittance_controller.launch"
echo ""
echo "2. 测试力输入："
echo "   rosrun robot_hw_test test_force_input.py"
echo ""
echo "3. 检查日志输出："
echo "   查看终端中的调试信息"
echo ""
echo "4. 如果机器人仍然不动，可能的原因："
echo "   - 导纳参数设置不当（质量、阻尼、刚度）"
echo "   - 力输入太小或滤波过度"
echo "   - 安全限制过于严格"
echo "   - 硬件接口问题"
echo ""
echo "=== 调试完成 ==="
