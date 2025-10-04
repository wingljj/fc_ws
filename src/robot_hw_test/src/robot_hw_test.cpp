#include "robot_hw_test/robot_hw_test.h"//包含自定义头文件

namespace robot_hw_test_ns

{

    MyRobotInterface::MyRobotInterface()

    {

        ;

    }

    MyRobotInterface::MyRobotInterface(ros::NodeHandle& nh)

    {


        ;

    }


    MyRobotInterface::~MyRobotInterface()

    {;}

//构造函数我都没管他


//init函数

   bool  MyRobotInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)

    {

        bool ret = robot_hw_nh.getParam("/mylink/robot_hw_test/joints", joint_name);//从参数服务器中获取name, 也就是从config文件夹下的yaml文件中获取参数,这个yaml文件我在后面会给出来,大家可以按照规则自己编写里面的内容,补充一点c++编程小知识,这里的 joint_name是一个vector类型的变量,直接写这个名字,返回的是这个变量第一个数的指针,那么这个getparam的第二个参数实际上是一个传出参数,将获取到的内容赋值给 joint_name变量.

        ROS_ERROR("getParam ret= %d",ret);//这里纯粹是调试用的,前期调bug的时候看看是不是运行正常

       

        num_joints = joint_name.size();//获取joint有几个

        for (size_t i = 0; i < num_joints; i++)

        {

            ROS_ERROR("jointname=%s",joint_name[i].c_str());//打印一下,当初调试用的

        }


        joint_position_state.resize(num_joints);//将下面的这些double类型的vector 的大小重构一下

    joint_velocity_state.resize(num_joints);

    joint_effort_state.resize(num_joints);

    joint_effort_command.resize(num_joints);



    for(int i=0; i<num_joints; i++)

    {

        //State

        hardware_interface::JointStateHandle jointStateHandle(joint_name[i].c_str(), &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);

//这里是重头戏,将对应的jointname和对应的位置,速度,力 绑定到 jointStateHandle中去

        joint_state_interface.registerHandle(jointStateHandle);


        ROS_INFO("joint_name[%d].c_str()=%s",i,jointStateHandle.getName().c_str());


        //Effort

        hardware_interface::JointHandle jointEffortHandle(joint_state_interface.getHandle(joint_name[i]), &joint_effort_command[i]);

        effort_joint_interface.registerHandle(jointEffortHandle);

//同理

    }


    registerInterface(&joint_state_interface);          //将类中的接口注册到ros中


    registerInterface(&effort_joint_interface);        

    return true;

    }


void MyRobotInterface::read(const ros::Time& time, const ros::Duration& period)

{

    static int t =0 ;

    if (t<3)

    {

        ROS_ERROR("read is run");

    }

    t++;

   


    double temp = std::sin(time.toSec());

//本篇的重点在于ros_control和robot_hw的通讯,硬件抽象层与硬件的交互不作为重点,这里只要ros_control能正常读到发过去的反馈,就代表成功

    for(int i=0;i < num_joints;i++){

        joint_position_state[i]=temp+1;

        }

}


void MyRobotInterface::write(const ros::Time& time, const ros::Duration& period)

    {

        for(int i=0;i < num_joints;i++)

        {

         ROS_WARN("recive from controller joint %d is %f",i,joint_effort_command[i]);

        //这里打印一下通过接口获取到的roscontrol发来的数据,有变化的数据就代表成功

        }

  

    }

}

PLUGINLIB_EXPORT_CLASS(robot_hw_test_ns::MyRobotInterface, hardware_interface::RobotHW)