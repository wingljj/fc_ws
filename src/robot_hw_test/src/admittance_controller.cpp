#include <robot_hw_test/admittance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace robot_hw_test {

AdmittanceController::AdmittanceController()
  : n_joints_(0)
{
}

AdmittanceController::~AdmittanceController()
{
}

bool AdmittanceController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh)
{
  nh_ = nh;
  position_joint_interface_ = hw->get<hardware_interface::PositionJointInterface>();
  
  // 获取关节名称列表
  std::vector<std::string> joint_names;
  if (!nh_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given in namespace: '%s'", nh_.getNamespace().c_str());
    return false;
  }
  
  n_joints_ = joint_names.size();
  
  if (n_joints_ == 0)
  {
    ROS_ERROR("No joints provided in the joint list");
    return false;
  }
  
  // 调整向量大小
  joints_.resize(n_joints_);
  current_joint_positions_.resize(n_joints_);
  current_joint_velocities_.resize(n_joints_);
  desired_joint_positions_.resize(n_joints_);
  desired_joint_velocities_.resize(n_joints_);
  desired_joint_accelerations_.resize(n_joints_);
  joint_torques_.resize(n_joints_);
  jacobian_.resize(n_joints_);
  
  // 初始化每个关节
  for (unsigned int i = 0; i < n_joints_; i++)
  {
    // 获取关节句柄
    try
    {
      joints_[i] = position_joint_interface_->getHandle(joint_names[i]);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown while getting joint handle: " << e.what());
      return false;
    }
  }
  
  // 加载机器人模型
  if (!loadRobotModel())
  {
    ROS_ERROR("Failed to load robot model");
    return false;
  }
  
  // 初始化导纳参数
  initializeAdmittanceParams();
  
  // 创建ROS订阅者和发布者
  external_force_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("external_force", 1, 
                                                                   &AdmittanceController::externalForceCallback, this);
  target_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("target_pose", 1,
                                                              &AdmittanceController::targetPoseCallback, this);
  
  current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("filtered_wrench", 1);
  
  // 初始化滤波器缓冲区
  for (int i = 0; i < 6; i++)
  {
    force_filter_buffer_[i].resize(FILTER_BUFFER_SIZE, 0.0);
    velocity_filter_buffer_[i].resize(FILTER_BUFFER_SIZE, 0.0);
  }
  
  ROS_INFO("AdmittanceController initialized successfully with %d joints", n_joints_);
  return true;
}

void AdmittanceController::initializeAdmittanceParams()
{
  // 初始化导纳参数矩阵
  admittance_params_.mass_matrix = Eigen::Matrix<double, 6, 6>::Identity();
  admittance_params_.damping_matrix = Eigen::Matrix<double, 6, 6>::Identity();
  admittance_params_.stiffness_matrix = Eigen::Matrix<double, 6, 6>::Identity();
  
  // 从参数服务器加载导纳参数
  // 质量矩阵参数
  std::vector<double> mass_diag;
  if (nh_.getParam("admittance/mass_diagonal", mass_diag) && mass_diag.size() == 6)
  {
    for (int i = 0; i < 6; i++)
    {
      admittance_params_.mass_matrix(i, i) = mass_diag[i];
    }
  }
  else
  {
    // 默认值
    admittance_params_.mass_matrix(0, 0) = 1.0;  // x
    admittance_params_.mass_matrix(1, 1) = 1.0;  // y
    admittance_params_.mass_matrix(2, 2) = 1.0;  // z
    admittance_params_.mass_matrix(3, 3) = 0.1;  // rx
    admittance_params_.mass_matrix(4, 4) = 0.1;  // ry
    admittance_params_.mass_matrix(5, 5) = 0.1;  // rz
  }
  
  // 阻尼矩阵参数
  std::vector<double> damping_diag;
  if (nh_.getParam("admittance/damping_diagonal", damping_diag) && damping_diag.size() == 6)
  {
    for (int i = 0; i < 6; i++)
    {
      admittance_params_.damping_matrix(i, i) = damping_diag[i];
    }
  }
  else
  {
    // 默认值
    admittance_params_.damping_matrix(0, 0) = 10.0;  // x
    admittance_params_.damping_matrix(1, 1) = 10.0;  // y
    admittance_params_.damping_matrix(2, 2) = 10.0;  // z
    admittance_params_.damping_matrix(3, 3) = 1.0;   // rx
    admittance_params_.damping_matrix(4, 4) = 1.0;   // ry
    admittance_params_.damping_matrix(5, 5) = 1.0;   // rz
  }
  
  // 刚度矩阵参数
  std::vector<double> stiffness_diag;
  if (nh_.getParam("admittance/stiffness_diagonal", stiffness_diag) && stiffness_diag.size() == 6)
  {
    for (int i = 0; i < 6; i++)
    {
      admittance_params_.stiffness_matrix(i, i) = stiffness_diag[i];
    }
  }
  else
  {
    // 默认值
    admittance_params_.stiffness_matrix(0, 0) = 100.0;  // x
    admittance_params_.stiffness_matrix(1, 1) = 100.0;  // y
    admittance_params_.stiffness_matrix(2, 2) = 100.0;  // z
    admittance_params_.stiffness_matrix(3, 3) = 10.0;   // rx
    admittance_params_.stiffness_matrix(4, 4) = 10.0;   // ry
    admittance_params_.stiffness_matrix(5, 5) = 10.0;   // rz
  }
  
  // 力限制
  nh_.param("admittance/max_force", admittance_params_.max_force, 50.0);
  nh_.param("admittance/max_torque", admittance_params_.max_torque, 10.0);
  
  // 位置限制
  nh_.param("admittance/max_position_error", admittance_params_.max_position_error, 0.1);
  nh_.param("admittance/max_velocity", admittance_params_.max_velocity, 0.5);
  
  // 滤波参数
  nh_.param("admittance/force_filter_cutoff", admittance_params_.force_filter_cutoff, 10.0);
  nh_.param("admittance/velocity_filter_cutoff", admittance_params_.velocity_filter_cutoff, 5.0);
}

bool AdmittanceController::loadRobotModel()
{
  // 获取机器人描述参数
  std::string robot_description;
  if (!nh_.getParam("/robot_description", robot_description))
  {
    ROS_ERROR("Failed to get robot_description parameter");
    return false;
  }
  
  // 解析URDF
  if (!robot_model_.initString(robot_description))
  {
    ROS_ERROR("Failed to parse robot description");
    return false;
  }
  
  // 获取基座和末端执行器链接名称
  std::string base_link, end_effector_link;
  if (!nh_.getParam("base_link", base_link))
  {
    base_link = "base_link";
  }
  if (!nh_.getParam("end_effector_link", end_effector_link))
  {
    end_effector_link = "tool0";
  }
  
  // 构建运动学链
  if (!kdl_parser::treeFromUrdfModel(robot_model_, robot_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree from robot model");
    return false;
  }
  
  if (!robot_tree_.getChain(base_link, end_effector_link, robot_chain_))
  {
    ROS_ERROR("Failed to get chain from %s to %s", base_link.c_str(), end_effector_link.c_str());
    return false;
  }
  
  // 创建求解器
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jacobian_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));
  dyn_param_solver_.reset(new KDL::ChainDynParam(robot_chain_, KDL::Vector(0, 0, -9.81)));
  
  ROS_INFO("Robot model loaded successfully. Chain has %d segments", robot_chain_.getNrOfSegments());
  return true;
}

void AdmittanceController::starting(const ros::Time& time)
{
  // 初始化期望位置为当前位置
  for (unsigned int i = 0; i < n_joints_; i++)
  {
    current_joint_positions_(i) = joints_[i].getPosition();
    current_joint_velocities_(i) = joints_[i].getVelocity();
    desired_joint_positions_(i) = current_joint_positions_(i);
    desired_joint_velocities_(i) = 0.0;
    desired_joint_accelerations_(i) = 0.0;
  }
  
  // 计算当前末端执行器位姿
  updateKinematics();
  desired_pose_ = current_pose_;
  desired_velocity_ = KDL::Twist::Zero();
  desired_acceleration_ = KDL::Twist::Zero();
  
  // 初始化外部力
  external_wrench_ = KDL::Wrench::Zero();
  filtered_wrench_ = KDL::Wrench::Zero();
  
  ROS_INFO("AdmittanceController started");
}

void AdmittanceController::update(const ros::Time& time, const ros::Duration& period)
{
  // 更新当前关节状态
  for (unsigned int i = 0; i < n_joints_; i++)
  {
    current_joint_positions_(i) = joints_[i].getPosition();
    current_joint_velocities_(i) = joints_[i].getVelocity();
  }
  
  // 更新运动学
  updateKinematics();
  
  // 获取外部力（线程安全）
  KDL::Wrench* wrench_from_buffer = wrench_buffer_.readFromRT();
  if (wrench_from_buffer)
  {
    external_wrench_ = *wrench_from_buffer;
  }
  
  // 获取目标位姿（线程安全）
  KDL::Frame* target_pose_from_buffer = target_pose_buffer_.readFromRT();
  if (target_pose_from_buffer)
  {
    // 这里可以添加目标位姿的处理逻辑
  }
  
  // 应用低通滤波
  applyLowPassFilter(external_wrench_, filtered_wrench_, admittance_params_.force_filter_cutoff, period);
  
  // 安全检查
  if (!checkSafetyLimits())
  {
    ROS_WARN("Safety limits exceeded, stopping controller");
    stopping(time);
    return;
  }
  
  // 更新导纳控制
  updateAdmittanceControl(period);
  
  // 计算所需扭矩
  computeRequiredTorque();
  
  // 应用控制扭矩
  for (unsigned int i = 0; i < n_joints_; i++)
  {
    joints_[i].setCommand(desired_joint_positions_(i));
  }
  
  // 发布当前状态
  publishCurrentState();
}

void AdmittanceController::stopping(const ros::Time& time)
{
  // 停止时保持当前位置
  for (unsigned int i = 0; i < n_joints_; i++)
  {
    joints_[i].setCommand(current_joint_positions_(i));
  }
  ROS_INFO("AdmittanceController stopped");
}

void AdmittanceController::updateKinematics()
{
  // 计算正向运动学
  fk_solver_->JntToCart(current_joint_positions_, current_pose_);
  
  // 计算雅可比矩阵
  jacobian_solver_->JntToJac(current_joint_positions_, jacobian_);
  
  // 计算当前速度（通过雅可比矩阵）
  KDL::Twist twist = jacobian_.data * current_joint_velocities_.data;
  current_velocity_ = twist;
}

void AdmittanceController::updateAdmittanceControl(const ros::Duration& period)
{
  // 计算位置和速度误差
  KDL::Frame pose_error = desired_pose_.Inverse() * current_pose_;
  KDL::Twist velocity_error = desired_velocity_ - current_velocity_;
  
  // 将位姿误差转换为twist
  KDL::Twist pose_error_twist;
  pose_error_twist.vel = pose_error.p;
  pose_error_twist.rot = KDL::Vector(pose_error.M.GetRot().x, pose_error.M.GetRot().y, pose_error.M.GetRot().z);
  
  // 应用力限制
  KDL::Wrench limited_wrench = filtered_wrench_;
  applyForceLimits(limited_wrench);
  
  // 导纳控制律：M * x_desired'' + D * x_desired' + K * x_desired = F_ext
  // 转换为：x_desired'' = M^(-1) * (F_ext - D * x_desired' - K * x_desired)
  
  // 将KDL类型转换为Eigen
  Eigen::Vector<double, 6> wrench_eigen, velocity_eigen, pose_error_eigen;
  wrench_eigen << limited_wrench.force.x, limited_wrench.force.y, limited_wrench.force.z,
                  limited_wrench.torque.x, limited_wrench.torque.y, limited_wrench.torque.z;
  velocity_eigen << desired_velocity_.vel.x, desired_velocity_.vel.y, desired_velocity_.vel.z,
                    desired_velocity_.rot.x, desired_velocity_.rot.y, desired_velocity_.rot.z;
  pose_error_eigen << pose_error_twist.vel.x, pose_error_twist.vel.y, pose_error_twist.vel.z,
                      pose_error_twist.rot.x, pose_error_twist.rot.y, pose_error_twist.rot.z;
  
  // 计算期望加速度
  Eigen::Vector<double, 6> desired_acceleration_eigen = 
    admittance_params_.mass_matrix.inverse() * 
    (wrench_eigen - admittance_params_.damping_matrix * velocity_eigen - 
     admittance_params_.stiffness_matrix * pose_error_eigen);
  
  // 转换回KDL类型
  desired_acceleration_.vel.x = desired_acceleration_eigen(0);
  desired_acceleration_.vel.y = desired_acceleration_eigen(1);
  desired_acceleration_.vel.z = desired_acceleration_eigen(2);
  desired_acceleration_.rot.x = desired_acceleration_eigen(3);
  desired_acceleration_.rot.y = desired_acceleration_eigen(4);
  desired_acceleration_.rot.z = desired_acceleration_eigen(5);
  
  // 积分得到期望速度和位置
  desired_velocity_ += desired_acceleration_ * period.toSec();
  desired_pose_.p += desired_velocity_.vel * period.toSec();
  
  // 应用速度限制
  applyVelocityLimits(desired_velocity_);
  
  // 应用位置限制
  applyPositionLimits(desired_pose_);
}

void AdmittanceController::computeRequiredTorque()
{
  // 计算位置和速度误差
  KDL::Frame pose_error = desired_pose_.Inverse() * current_pose_;
  KDL::Twist velocity_error = desired_velocity_ - current_velocity_;
  
  // 将位姿误差转换为twist
  KDL::Twist pose_error_twist;
  pose_error_twist.vel = pose_error.p;
  pose_error_twist.rot = KDL::Vector(pose_error.M.GetRot().x, pose_error.M.GetRot().y, pose_error.M.GetRot().z);
  
  // 计算笛卡尔空间的期望力
  KDL::Wrench desired_wrench;
  desired_wrench.force.x = admittance_params_.stiffness_matrix(0, 0) * pose_error_twist.vel.x + 
                           admittance_params_.damping_matrix(0, 0) * velocity_error.vel.x;
  desired_wrench.force.y = admittance_params_.stiffness_matrix(1, 1) * pose_error_twist.vel.y + 
                           admittance_params_.damping_matrix(1, 1) * velocity_error.vel.y;
  desired_wrench.force.z = admittance_params_.stiffness_matrix(2, 2) * pose_error_twist.vel.z + 
                           admittance_params_.damping_matrix(2, 2) * velocity_error.vel.z;
  desired_wrench.torque.x = admittance_params_.stiffness_matrix(3, 3) * pose_error_twist.rot.x + 
                            admittance_params_.damping_matrix(3, 3) * velocity_error.rot.x;
  desired_wrench.torque.y = admittance_params_.stiffness_matrix(4, 4) * pose_error_twist.rot.y + 
                            admittance_params_.damping_matrix(4, 4) * velocity_error.rot.y;
  desired_wrench.torque.z = admittance_params_.stiffness_matrix(5, 5) * pose_error_twist.rot.z + 
                            admittance_params_.damping_matrix(5, 5) * velocity_error.rot.z;
  
  // 通过雅可比转置将笛卡尔力转换为关节力矩
  joint_torques_.data = jacobian_.data.transpose() * desired_wrench.data;
  
  // 这里应该将关节力矩转换为位置命令
  // 为了简化，我们直接使用期望位置
  // 在实际应用中，这里需要实现力矩到位置的转换
}

void AdmittanceController::applyLowPassFilter(const KDL::Wrench& input, KDL::Wrench& output, 
                                             double cutoff_freq, const ros::Duration& period)
{
  // 简单的移动平均滤波器
  double alpha = 2.0 * M_PI * cutoff_freq * period.toSec();
  alpha = std::min(alpha, 1.0);
  
  output.force.x = (1.0 - alpha) * output.force.x + alpha * input.force.x;
  output.force.y = (1.0 - alpha) * output.force.y + alpha * input.force.y;
  output.force.z = (1.0 - alpha) * output.force.z + alpha * input.force.z;
  output.torque.x = (1.0 - alpha) * output.torque.x + alpha * input.torque.x;
  output.torque.y = (1.0 - alpha) * output.torque.y + alpha * input.torque.y;
  output.torque.z = (1.0 - alpha) * output.torque.z + alpha * input.torque.z;
}

void AdmittanceController::applyLowPassFilter(const KDL::Twist& input, KDL::Twist& output, 
                                             double cutoff_freq, const ros::Duration& period)
{
  // 简单的移动平均滤波器
  double alpha = 2.0 * M_PI * cutoff_freq * period.toSec();
  alpha = std::min(alpha, 1.0);
  
  output.vel.x = (1.0 - alpha) * output.vel.x + alpha * input.vel.x;
  output.vel.y = (1.0 - alpha) * output.vel.y + alpha * input.vel.y;
  output.vel.z = (1.0 - alpha) * output.vel.z + alpha * input.vel.z;
  output.rot.x = (1.0 - alpha) * output.rot.x + alpha * input.rot.x;
  output.rot.y = (1.0 - alpha) * output.rot.y + alpha * input.rot.y;
  output.rot.z = (1.0 - alpha) * output.rot.z + alpha * input.rot.z;
}

void AdmittanceController::externalForceCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  // 将ROS消息转换为KDL类型
  KDL::Wrench wrench;
  wrench.force.x = msg->wrench.force.x;
  wrench.force.y = msg->wrench.force.y;
  wrench.force.z = msg->wrench.force.z;
  wrench.torque.x = msg->wrench.torque.x;
  wrench.torque.y = msg->wrench.torque.y;
  wrench.torque.z = msg->wrench.torque.z;
  
  // 线程安全地写入缓冲区
  wrench_buffer_.writeFromNonRT(wrench);
}

void AdmittanceController::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // 将ROS消息转换为KDL类型
  KDL::Frame frame;
  tf::poseMsgToKDL(msg->pose, frame);
  
  // 线程安全地写入缓冲区
  target_pose_buffer_.writeFromNonRT(frame);
}

bool AdmittanceController::checkSafetyLimits()
{
  // 检查力限制
  double force_magnitude = sqrt(filtered_wrench_.force.x * filtered_wrench_.force.x +
                                filtered_wrench_.force.y * filtered_wrench_.force.y +
                                filtered_wrench_.force.z * filtered_wrench_.force.z);
  if (force_magnitude > admittance_params_.max_force)
  {
    ROS_WARN("Force limit exceeded: %f > %f", force_magnitude, admittance_params_.max_force);
    return false;
  }
  
  // 检查力矩限制
  double torque_magnitude = sqrt(filtered_wrench_.torque.x * filtered_wrench_.torque.x +
                                 filtered_wrench_.torque.y * filtered_wrench_.torque.y +
                                 filtered_wrench_.torque.z * filtered_wrench_.torque.z);
  if (torque_magnitude > admittance_params_.max_torque)
  {
    ROS_WARN("Torque limit exceeded: %f > %f", torque_magnitude, admittance_params_.max_torque);
    return false;
  }
  
  // 检查速度限制
  double velocity_magnitude = sqrt(desired_velocity_.vel.x * desired_velocity_.vel.x +
                                   desired_velocity_.vel.y * desired_velocity_.vel.y +
                                   desired_velocity_.vel.z * desired_velocity_.vel.z);
  if (velocity_magnitude > admittance_params_.max_velocity)
  {
    ROS_WARN("Velocity limit exceeded: %f > %f", velocity_magnitude, admittance_params_.max_velocity);
    return false;
  }
  
  return true;
}

void AdmittanceController::applyForceLimits(KDL::Wrench& wrench)
{
  // 限制力的大小
  double force_magnitude = sqrt(wrench.force.x * wrench.force.x +
                                wrench.force.y * wrench.force.y +
                                wrench.force.z * wrench.force.z);
  if (force_magnitude > admittance_params_.max_force)
  {
    double scale = admittance_params_.max_force / force_magnitude;
    wrench.force.x *= scale;
    wrench.force.y *= scale;
    wrench.force.z *= scale;
  }
  
  // 限制力矩的大小
  double torque_magnitude = sqrt(wrench.torque.x * wrench.torque.x +
                                 wrench.torque.y * wrench.torque.y +
                                 wrench.torque.z * wrench.torque.z);
  if (torque_magnitude > admittance_params_.max_torque)
  {
    double scale = admittance_params_.max_torque / torque_magnitude;
    wrench.torque.x *= scale;
    wrench.torque.y *= scale;
    wrench.torque.z *= scale;
  }
}

void AdmittanceController::applyVelocityLimits(KDL::Twist& twist)
{
  // 限制线速度
  double vel_magnitude = sqrt(twist.vel.x * twist.vel.x +
                              twist.vel.y * twist.vel.y +
                              twist.vel.z * twist.vel.z);
  if (vel_magnitude > admittance_params_.max_velocity)
  {
    double scale = admittance_params_.max_velocity / vel_magnitude;
    twist.vel.x *= scale;
    twist.vel.y *= scale;
    twist.vel.z *= scale;
  }
  
  // 限制角速度（使用较小的限制）
  double rot_magnitude = sqrt(twist.rot.x * twist.rot.x +
                              twist.rot.y * twist.rot.y +
                              twist.rot.z * twist.rot.z);
  double max_rot_velocity = admittance_params_.max_velocity * 0.1; // 角速度限制为线速度的10%
  if (rot_magnitude > max_rot_velocity)
  {
    double scale = max_rot_velocity / rot_magnitude;
    twist.rot.x *= scale;
    twist.rot.y *= scale;
    twist.rot.z *= scale;
  }
}

void AdmittanceController::applyPositionLimits(KDL::Frame& pose)
{
  // 这里可以添加位置限制逻辑
  // 例如工作空间限制等
}

void AdmittanceController::publishCurrentState()
{
  // 发布当前位姿
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "base_link";
  tf::poseKDLToMsg(current_pose_, pose_msg.pose);
  current_pose_pub_.publish(pose_msg);
  
  // 发布关节状态
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < n_joints_; i++)
  {
    joint_state_msg.name.push_back(joints_[i].getName());
    joint_state_msg.position.push_back(current_joint_positions_(i));
    joint_state_msg.velocity.push_back(current_joint_velocities_(i));
  }
  joint_state_pub_.publish(joint_state_msg);
  
  // 发布滤波后的力
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = ros::Time::now();
  wrench_msg.header.frame_id = "tool0";
  wrench_msg.wrench.force.x = filtered_wrench_.force.x;
  wrench_msg.wrench.force.y = filtered_wrench_.force.y;
  wrench_msg.wrench.force.z = filtered_wrench_.force.z;
  wrench_msg.wrench.torque.x = filtered_wrench_.torque.x;
  wrench_msg.wrench.torque.y = filtered_wrench_.torque.y;
  wrench_msg.wrench.torque.z = filtered_wrench_.torque.z;
  wrench_pub_.publish(wrench_msg);
}

} // namespace robot_hw_test

PLUGINLIB_EXPORT_CLASS(robot_hw_test::AdmittanceController, controller_interface::ControllerBase)