#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
测试力输入脚本
用于验证导纳控制器是否能正确接收和处理力输入
"""

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header

def test_force_input():
    rospy.init_node('test_force_input', anonymous=True)
    
    # 创建发布者
    force_pub = rospy.Publisher('/external_force', WrenchStamped, queue_size=1)
    
    # 等待订阅者连接
    rospy.sleep(1.0)
    
    rospy.loginfo("开始测试力输入...")
    
    rate = rospy.Rate(10)  # 10Hz
    time_counter = 0.0
    
    while not rospy.is_shutdown():
        # 创建力消息
        wrench_msg = WrenchStamped()
        wrench_msg.header = Header()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = "tool0"
        
        # 生成测试力 - 正弦波
        amplitude = 5.0  # 5N
        frequency = 0.5  # 0.5Hz
        
        wrench_msg.wrench.force.x = amplitude * np.sin(2 * np.pi * frequency * time_counter)
        wrench_msg.wrench.force.y = 0.0
        wrench_msg.wrench.force.z = 0.0
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0
        
        # 发布消息
        force_pub.publish(wrench_msg)
        
        # 打印调试信息
        if int(time_counter * 10) % 10 == 0:  # 每秒打印一次
            rospy.loginfo("Published force: Fx=%.3f N", wrench_msg.wrench.force.x)
        
        time_counter += 0.1  # 10Hz
        rate.sleep()

if __name__ == '__main__':
    try:
        test_force_input()
    except rospy.ROSInterruptException:
        rospy.loginfo("力输入测试已停止")
