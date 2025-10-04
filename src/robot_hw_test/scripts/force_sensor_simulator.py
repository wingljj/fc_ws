#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
力传感器模拟器
用于测试导纳控制器的力输入功能
"""

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header

class ForceSensorSimulator:
    def __init__(self):
        rospy.init_node('force_sensor_simulator', anonymous=True)
        
        # 参数
        self.publish_rate = rospy.get_param('~publish_rate', 100.0)
        self.force_topic = rospy.get_param('~force_topic', '/external_force')
        
        # 发布者
        self.wrench_pub = rospy.Publisher(self.force_topic, WrenchStamped, queue_size=1)
        
        # 力传感器参数
        self.force_noise_std = 0.1  # 力噪声标准差
        self.torque_noise_std = 0.01  # 力矩噪声标准差
        
        # 模拟力模式
        self.force_mode = rospy.get_param('~force_mode', 'sinusoidal')  # 'sinusoidal', 'constant', 'step'
        self.force_amplitude = rospy.get_param('~force_amplitude', 5.0)
        self.force_frequency = rospy.get_param('~force_frequency', 0.5)
        
        # 时间计数器
        self.time_counter = 0.0
        
        rospy.loginfo("力传感器模拟器已启动")
        rospy.loginfo("发布频率: {} Hz".format(self.publish_rate))
        rospy.loginfo("力模式: {}".format(self.force_mode))
        rospy.loginfo("力幅值: {} N".format(self.force_amplitude))
        
    def generate_force(self):
        """生成模拟力"""
        if self.force_mode == 'sinusoidal':
            # 正弦波力
            force_x = self.force_amplitude * np.sin(2 * np.pi * self.force_frequency * self.time_counter)
            force_y = self.force_amplitude * 0.5 * np.sin(2 * np.pi * self.force_frequency * self.time_counter + np.pi/2)
            force_z = self.force_amplitude * 0.3 * np.sin(2 * np.pi * self.force_frequency * self.time_counter + np.pi)
            
        elif self.force_mode == 'constant':
            # 恒定力
            force_x = self.force_amplitude
            force_y = 0.0
            force_z = 0.0
            
        elif self.force_mode == 'step':
            # 阶跃力
            if int(self.time_counter * self.force_frequency) % 2 == 0:
                force_x = self.force_amplitude
                force_y = 0.0
                force_z = 0.0
            else:
                force_x = 0.0
                force_y = 0.0
                force_z = 0.0
        else:
            # 默认无力
            force_x = 0.0
            force_y = 0.0
            force_z = 0.0
        
        # 添加噪声
        force_x += np.random.normal(0, self.force_noise_std)
        force_y += np.random.normal(0, self.force_noise_std)
        force_z += np.random.normal(0, self.force_noise_std)
        
        # 生成力矩（较小）
        torque_x = np.random.normal(0, self.torque_noise_std)
        torque_y = np.random.normal(0, self.torque_noise_std)
        torque_z = np.random.normal(0, self.torque_noise_std)
        
        return force_x, force_y, force_z, torque_x, torque_y, torque_z
    
    def publish_wrench(self):
        """发布力/力矩消息"""
        # 生成力
        fx, fy, fz, tx, ty, tz = self.generate_force()
        
        # 创建消息
        wrench_msg = WrenchStamped()
        wrench_msg.header = Header()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = "tool0"
        
        # 设置力
        wrench_msg.wrench.force.x = fx
        wrench_msg.wrench.force.y = fy
        wrench_msg.wrench.force.z = fz
        
        # 设置力矩
        wrench_msg.wrench.torque.x = tx
        wrench_msg.wrench.torque.y = ty
        wrench_msg.wrench.torque.z = tz
        
        # 发布消息
        self.wrench_pub.publish(wrench_msg)
        
        # 更新时间计数器
        self.time_counter += 1.0 / self.publish_rate
    
    def run(self):
        """运行模拟器"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            self.publish_wrench()
            rate.sleep()

if __name__ == '__main__':
    try:
        simulator = ForceSensorSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("力传感器模拟器已停止")
