#include <iostream>

#include <hardware_interface/joint_state_interface.h>

#include <hardware_interface/joint_command_interface.h>

#include <hardware_interface/robot_hw.h>

#include <pluginlib/class_list_macros.hpp>

#include <ros/ros.h>



namespace robot_hw_test_ns

{


//继承robothw类

    class MyRobotInterface : public hardware_interface::RobotHW

    {


    public:

    MyRobotInterface();//构造函数

    MyRobotInterface(ros::NodeHandle& nh);//构造函数,传入ros句柄,应用ros的资源

    ~MyRobotInterface();//析构函数

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);//初始化代码

    void read(const ros::Time& time, const ros::Duration& period);//将硬件资源读取到接口中,发送给控制器

    void write(const ros::Time& time, const ros::Duration& period);//将控制器给出的指令发送给硬件

    private:

    ros::NodeHandle nh_;//ros句柄

    //interfaces

    hardware_interface::JointStateInterface joint_state_interface;//硬件接口,状态反馈

    hardware_interface::EffortJointInterface effort_joint_interface;//硬件接口,力矩接口

   

    int num_joints;//

    std::vector<std::string> joint_name;


    std::vector<double> joint_position_state;//位置状态

    std::vector<double> joint_velocity_state;//速度状态

    std::vector<double> joint_effort_state;//力矩状态


    std::vector<double> joint_effort_command;//力矩指令


    };

}


