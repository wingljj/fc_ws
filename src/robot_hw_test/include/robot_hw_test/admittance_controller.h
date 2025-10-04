#ifndef ROBOT_HW_TEST_ADMITTANCE_CONTROLLER_H
#define ROBOT_HW_TEST_ADMITTANCE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <controller_interface/multi_interface_controller.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Dense>

namespace robot_hw_test {

class AdmittanceController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
{
public:
  AdmittanceController();
  ~AdmittanceController();

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  // 关节相关变量
  std::vector<hardware_interface::JointHandle> joints_;
  unsigned int n_joints_;
  hardware_interface::PositionJointInterface* position_joint_interface_;
  
  // 机器人模型相关
  urdf::Model robot_model_;
  KDL::Tree robot_tree_;
  KDL::Chain robot_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_solver_;
  
  // 导纳控制参数
  struct AdmittanceParams {
    // 笛卡尔空间导纳参数
    Eigen::Matrix<double, 6, 6> mass_matrix;      // 质量矩阵
    Eigen::Matrix<double, 6, 6> damping_matrix;   // 阻尼矩阵
    Eigen::Matrix<double, 6, 6> stiffness_matrix; // 刚度矩阵
    
    // 力限制
    double max_force;
    double max_torque;
    
    // 位置限制
    double max_position_error;
    double max_velocity;
    
    // 滤波参数
    double force_filter_cutoff;
    double velocity_filter_cutoff;
  };
  
  AdmittanceParams admittance_params_;
  
  // 状态变量
  KDL::JntArray current_joint_positions_;
  KDL::JntArray current_joint_velocities_;
  KDL::JntArray desired_joint_positions_;
  KDL::JntArray desired_joint_velocities_;
  KDL::JntArray desired_joint_accelerations_;
  
  KDL::Frame current_pose_;
  KDL::Twist current_velocity_;
  KDL::Frame desired_pose_;
  KDL::Twist desired_velocity_;
  KDL::Twist desired_acceleration_;
  
  KDL::Wrench external_wrench_;
  KDL::Wrench filtered_wrench_;
  KDL::JntArray joint_torques_;
  
  // 雅可比矩阵
  KDL::Jacobian jacobian_;
  
  // 滤波相关
  std::vector<double> force_filter_buffer_[6];
  std::vector<double> velocity_filter_buffer_[6];
  static const size_t FILTER_BUFFER_SIZE = 10;
  
  // ROS 相关
  ros::NodeHandle nh_;
  ros::Subscriber external_force_sub_;
  ros::Subscriber target_pose_sub_;
  ros::Publisher current_pose_pub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher wrench_pub_;
  
  tf::TransformListener tf_listener_;
  
  // 线程安全缓冲区
  realtime_tools::RealtimeBuffer<KDL::Wrench> wrench_buffer_;
  realtime_tools::RealtimeBuffer<KDL::Frame> target_pose_buffer_;
  
  // 辅助函数
  void updateAdmittanceControl(const ros::Duration& period);
  void computeRequiredTorque();
  void updateKinematics();
  void applyLowPassFilter(const KDL::Wrench& input, KDL::Wrench& output, double cutoff_freq, const ros::Duration& period);
  void applyLowPassFilter(const KDL::Twist& input, KDL::Twist& output, double cutoff_freq, const ros::Duration& period);
  bool loadRobotModel();
  void initializeAdmittanceParams();
  void publishCurrentState();
  
  // 回调函数
  void externalForceCallback(const geometry_msgs::WrenchStampedConstPtr& msg);
  void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  
  // 安全检查和限制
  bool checkSafetyLimits();
  void applyForceLimits(KDL::Wrench& wrench);
  void applyVelocityLimits(KDL::Twist& twist);
  void applyPositionLimits(KDL::Frame& pose);
};

} // namespace robot_hw_test

#endif // ROBOT_HW_TEST_ADMITTANCE_CONTROLLER_H