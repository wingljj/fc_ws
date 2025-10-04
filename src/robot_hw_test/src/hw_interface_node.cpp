#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <iostream>
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "robot_hw_test/robot_hw_test.h"
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "hw_interface_node");

    //节点名称...

    ros::AsyncSpinner spinner(1);
    spinner.start();//好象是线程开启

    ros::NodeHandle nh;//ros句柄

    robot_hw_test_ns::MyRobotInterface hw;//生成我们自定义的robothw类


    int ret = hw.init(nh,nh);


    ROS_INFO("hw init ret=%d",ret);


    controller_manager::ControllerManager cm(&hw,nh);//生成控制器管理器,这里我是直接抄的例程,其实并不是很懂


    sleep(1);

    ros::Duration period(1.0/125);//设定循环周期,注意这里周期是多少,控制器的周期就会跟着变

    ROS_INFO("2joint hw run");


    while (ros::ok())

    {

        hw.read(ros::Time::now(),period);//读取数据

        cm.update(ros::Time::now(),period);//将控制器管理者管理的控制器进入update函数

        hw.write(ros::Time::now(),period);//将ros_control给的指令转发给硬件

        period.sleep();

    }

    spinner.stop();

    return 0;

}