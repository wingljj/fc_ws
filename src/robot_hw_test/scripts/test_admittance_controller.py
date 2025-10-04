#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
导纳控制器测试脚本
用于验证导纳控制器的基本功能
"""

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import time

class AdmittanceControllerTester:
    def __init__(self):
        rospy.init_node('admittance_controller_tester', anonymous=True)
        
        # 订阅者
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.filtered_wrench_sub = rospy.Subscriber('/filtered_wrench', WrenchStamped, self.filtered_wrench_callback)
        
        # 发布者
        self.force_pub = rospy.Publisher('/external_force', WrenchStamped, queue_size=1)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=1)
        
        # 数据记录
        self.current_poses = []
        self.joint_states = []
        self.filtered_wrenches = []
        self.timestamps = []
        
        # 测试参数
        self.test_duration = 30.0  # 测试持续时间（秒）
        self.start_time = None
        
        rospy.loginfo("导纳控制器测试器已启动")
        
    def current_pose_callback(self, msg):
        """当前位姿回调"""
        if self.start_time is not None:
            current_time = rospy.Time.now().to_sec() - self.start_time
            self.current_poses.append([
                current_time,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ])
    
    def joint_state_callback(self, msg):
        """关节状态回调"""
        if self.start_time is not None:
            current_time = rospy.Time.now().to_sec() - self.start_time
            positions = [0.0] * 6
            velocities = [0.0] * 6
            
            for i, name in enumerate(msg.name):
                if i < 6:  # 只记录前6个关节
                    positions[i] = msg.position[i] if i < len(msg.position) else 0.0
                    velocities[i] = msg.velocity[i] if i < len(msg.velocity) else 0.0
            
            self.joint_states.append([current_time] + positions + velocities)
    
    def filtered_wrench_callback(self, msg):
        """滤波后力/力矩回调"""
        if self.start_time is not None:
            current_time = rospy.Time.now().to_sec() - self.start_time
            self.filtered_wrenches.append([
                current_time,
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z
            ])
    
    def publish_test_force(self, t):
        """发布测试力"""
        # 正弦波力测试
        amplitude = 10.0
        frequency = 0.5
        
        force_x = amplitude * np.sin(2 * np.pi * frequency * t)
        force_y = amplitude * 0.5 * np.sin(2 * np.pi * frequency * t + np.pi/2)
        force_z = 0.0
        
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = "tool0"
        wrench_msg.wrench.force.x = force_x
        wrench_msg.wrench.force.y = force_y
        wrench_msg.wrench.force.z = force_z
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0
        
        self.force_pub.publish(wrench_msg)
    
    def publish_target_pose(self, t):
        """发布目标位姿"""
        # 静态目标位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = 0.5
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.3
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.target_pose_pub.publish(pose_msg)
    
    def run_test(self):
        """运行测试"""
        rospy.loginfo("开始导纳控制器测试...")
        rospy.loginfo("测试持续时间: {} 秒".format(self.test_duration))
        
        # 等待控制器启动
        rospy.sleep(2.0)
        
        self.start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)  # 50Hz
        
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - self.start_time) < self.test_duration:
            current_time = rospy.Time.now().to_sec() - self.start_time
            
            # 发布测试力
            self.publish_test_force(current_time)
            
            # 发布目标位姿
            self.publish_target_pose(current_time)
            
            rate.sleep()
        
        rospy.loginfo("测试完成，开始分析数据...")
        self.analyze_results()
    
    def analyze_results(self):
        """分析测试结果"""
        if not self.current_poses or not self.joint_states or not self.filtered_wrenches:
            rospy.logwarn("没有收集到足够的数据进行分析")
            return
        
        # 转换为numpy数组
        poses = np.array(self.current_poses)
        joints = np.array(self.joint_states)
        wrenches = np.array(self.filtered_wrenches)
        
        # 创建图表
        fig, axes = plt.subplots(3, 2, figsize=(15, 10))
        fig.suptitle('导纳控制器测试结果', fontsize=16)
        
        # 位置轨迹
        axes[0, 0].plot(poses[:, 0], poses[:, 1], label='X')
        axes[0, 0].plot(poses[:, 0], poses[:, 2], label='Y')
        axes[0, 0].plot(poses[:, 0], poses[:, 3], label='Z')
        axes[0, 0].set_title('末端执行器位置')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('位置 (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # 关节位置
        for i in range(6):
            axes[0, 1].plot(joints[:, 0], joints[:, i+1], label='Joint {}'.format(i+1))
        axes[0, 1].set_title('关节位置')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].set_ylabel('位置 (rad)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # 力输入
        axes[1, 0].plot(wrenches[:, 0], wrenches[:, 1], label='Fx')
        axes[1, 0].plot(wrenches[:, 0], wrenches[:, 2], label='Fy')
        axes[1, 0].plot(wrenches[:, 0], wrenches[:, 3], label='Fz')
        axes[1, 0].set_title('滤波后力输入')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('力 (N)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # 力矩输入
        axes[1, 1].plot(wrenches[:, 0], wrenches[:, 4], label='Tx')
        axes[1, 1].plot(wrenches[:, 0], wrenches[:, 5], label='Ty')
        axes[1, 1].plot(wrenches[:, 0], wrenches[:, 6], label='Tz')
        axes[1, 1].set_title('滤波后力矩输入')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('力矩 (Nm)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        # 关节速度
        for i in range(6):
            axes[2, 0].plot(joints[:, 0], joints[:, i+7], label='Joint {}'.format(i+1))
        axes[2, 0].set_title('关节速度')
        axes[2, 0].set_xlabel('时间 (s)')
        axes[2, 0].set_ylabel('速度 (rad/s)')
        axes[2, 0].legend()
        axes[2, 0].grid(True)
        
        # 位置误差
        target_x = 0.5
        target_y = 0.0
        target_z = 0.3
        position_error = np.sqrt((poses[:, 1] - target_x)**2 + 
                                (poses[:, 2] - target_y)**2 + 
                                (poses[:, 3] - target_z)**2)
        axes[2, 1].plot(poses[:, 0], position_error)
        axes[2, 1].set_title('位置跟踪误差')
        axes[2, 1].set_xlabel('时间 (s)')
        axes[2, 1].set_ylabel('误差 (m)')
        axes[2, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig('/tmp/admittance_controller_test_results.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # 打印统计信息
        rospy.loginfo("=== 测试结果统计 ===")
        rospy.loginfo("平均位置误差: {:.4f} m".format(np.mean(position_error)))
        rospy.loginfo("最大位置误差: {:.4f} m".format(np.max(position_error)))
        rospy.loginfo("位置误差标准差: {:.4f} m".format(np.std(position_error)))
        
        # 力响应分析
        max_force = np.max(np.sqrt(wrenches[:, 1]**2 + wrenches[:, 2]**2 + wrenches[:, 3]**2))
        rospy.loginfo("最大力输入: {:.4f} N".format(max_force))
        
        rospy.loginfo("测试结果图表已保存到: /tmp/admittance_controller_test_results.png")

if __name__ == '__main__':
    try:
        tester = AdmittanceControllerTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("导纳控制器测试已停止")
