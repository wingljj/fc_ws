#include <std_msgs/Float64MultiArray.h>
#include "robot_hw_test/joint_controller_test.h"
// KDL
#include <cmath>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
// URDF
#include <urdf/model.h>
#include <urdf_model/joint.h>

namespace robot_hw_test_ns
{
joint_controller_test::joint_controller_test(/* args */)
{
    
}
joint_controller_test::~joint_controller_test()
{
}
bool joint_controller_test::init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh)
{
    std::string robot_description;
    urdf::Model robot_model;
    KDL::Tree   robot_tree;
    if (!ros::param::search("robot_description", robot_description))
    {
        ROS_ERROR_STREAM("Searched enclosing namespaces for robot_description but nothing found");
        return false;
    }
    if (!nh.getParam(robot_description, robot_description))
    {
        ROS_ERROR_STREAM("Failed to load " << robot_description << " from parameter server");
        return false;
    }
    if (!nh.getParam("robot_base_link",m_robot_base_link))
    {
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/robot_base_link" << " from parameter server");
        return false;
    }
    if (!nh.getParam("end_effector_link",m_end_effector_link))
    {
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/end_effector_link" << " from parameter server");
        return false;
    }

    
    // Build a kinematic chain of the robot
    if (!robot_model.initString(robot_description))
    {
        ROS_ERROR("Failed to parse urdf model from 'robot_description'");
        return false;
    }
    
    // if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree))
    // {
    //     const std::string error = ""
    //     "Failed to parse KDL tree from urdf model";
    //     ROS_ERROR_STREAM(error);
    //     throw std::runtime_error(error);
    // }

    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
    {
        const std::string error = "Failed to parse KDL tree from URDF model";
        ROS_ERROR_STREAM(error);
        // 打印 URDF 内容长度等信息用于调试
        ROS_ERROR_STREAM("URDF content length: " << robot_description.length());
        throw std::runtime_error(error);
    }


    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names)) {
        ROS_ERROR("Agv_Controller: Invalid or no joint_names parameters provided, aborting controller init!");
        return false;
    }   
    

    auto* joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
    for (uint8_t i = 0; i < joint_names.size(); i++) {
        joint_handles_.push_back(joint_interface->getHandle(joint_names[i]));
        ROS_ERROR_STREAM("controller info: my name is " << joint_handles_[i].getName());
    }
    joint_state_.resize(joint_names.size());

    // 添加话题订阅
    joint_commands_.resize(joint_names.size(), 0.0);
    joint_command_sub_ = nh.subscribe("joint_commands", 1, 
                                        &joint_controller_test::jointCommandCallback, this);

    return true;

}


void joint_controller_test::starting(const ros::Time&)

{

ROS_WARN("controller starting");

}


void joint_controller_test::update(const ros::Time& time_now, const ros::Duration& period)
{

for (size_t i = 0; i < joint_handles_.size(); i++) {
        joint_handles_[i].setCommand(joint_commands_[i]);
        // ROS_WARN("receive from robot_hw feedback is %f", joint_handles_[i].getPosition());
    }
}

void joint_controller_test::jointCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != joint_commands_.size()) {
        ROS_WARN("Received command size (%lu) doesn't match joint count (%lu)", 
                msg->data.size(), joint_commands_.size());
        return;
    }
    joint_commands_ = msg->data;
}

} //namespace

PLUGINLIB_EXPORT_CLASS(robot_hw_test_ns::joint_controller_test,

controller_interface::ControllerBase)