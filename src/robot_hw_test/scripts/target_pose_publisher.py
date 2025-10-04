#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
目标位姿发布器
用于测试导纳控制器的目标位姿跟踪功能
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf.transformations as tf_trans

class TargetPosePublisher:
    def __init__(self):
        rospy.init_node('target_pose_publisher', anonymous=True)
        
        # 参数
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)
        self.target_pose_topic = rospy.get_param('~target_pose_topic', '/target_pose')
        
        # 发布者
        self.pose_pub = rospy.Publisher(self.target_pose_topic, PoseStamped, queue_size=1)
        
        # 目标位姿参数
        self.initial_position = [0.5, 0.0, 0.3]  # 初始位置 [x, y, z]
        self.initial_orientation = [0.0, 0.0, 0.0, 1.0]  # 初始四元数 [x, y, z, w]
        
        # 运动模式
        self.motion_mode = rospy.get_param('~motion_mode', 'static')  # 'static', 'circular', 'linear'
        self.motion_amplitude = rospy.get_param('~motion_amplitude', 0.1)
        self.motion_frequency = rospy.get_param('~motion_frequency', 0.2)
        
        # 时间计数器
        self.time_counter = 0.0
        
        rospy.loginfo("目标位姿发布器已启动")
        rospy.loginfo("发布频率: {} Hz".format(self.publish_rate))
        rospy.loginfo("运动模式: {}".format(self.motion_mode))
        
    def generate_target_pose(self):
        """生成目标位姿"""
        if self.motion_mode == 'static':
            # 静态位姿
            position = self.initial_position.copy()
            orientation = self.initial_orientation.copy()
            
        elif self.motion_mode == 'circular':
            # 圆形运动
            radius = self.motion_amplitude
            angle = 2 * np.pi * self.motion_frequency * self.time_counter
            
            position = [
                self.initial_position[0] + radius * np.cos(angle),
                self.initial_position[1] + radius * np.sin(angle),
                self.initial_position[2]
            ]
            orientation = self.initial_orientation.copy()
            
        elif self.motion_mode == 'linear':
            # 线性运动
            amplitude = self.motion_amplitude
            position = [
                self.initial_position[0] + amplitude * np.sin(2 * np.pi * self.motion_frequency * self.time_counter),
                self.initial_position[1],
                self.initial_position[2]
            ]
            orientation = self.initial_orientation.copy()
            
        else:
            # 默认静态位姿
            position = self.initial_position.copy()
            orientation = self.initial_orientation.copy()
        
        return position, orientation
    
    def publish_target_pose(self):
        """发布目标位姿"""
        # 生成目标位姿
        position, orientation = self.generate_target_pose()
        
        # 创建消息
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        
        # 设置位置
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        
        # 设置方向
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        
        # 发布消息
        self.pose_pub.publish(pose_msg)
        
        # 更新时间计数器
        self.time_counter += 1.0 / self.publish_rate
    
    def run(self):
        """运行发布器"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            self.publish_target_pose()
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TargetPosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("目标位姿发布器已停止")
