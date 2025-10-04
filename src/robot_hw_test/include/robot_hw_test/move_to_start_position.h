#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_example_controllers {

class MoveToStartPosition : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 6> q_start_{{0, 0, 0, 0, 0, 0}};
    double factor_{1.0};


  static constexpr double kDeltaQMotionFinished = 1e-6;
  std::array<double, 6> q_goal_{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2}};
   std::array<double, 6> delta_q_{{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> dq_max_sync_{{0, 0, 0, 0, 0}};
  std::array<double, 6> t_1_sync_{{0, 0, 0, 0, 0, 0}};
   std::array<double, 6> t_2_sync_{{0, 0, 0, 0, 0, 0}};
   std::array<double, 6> t_f_sync_{{0, 0, 0, 0, 0, 0}};
   std::array<double, 6> q_1_{{0, 0, 0, 0, 0, 0}};
   std::array<double, 6> t_d_{{0, 0, 0, 0, 0, 0}};
   std::array<int, 6> sign_delta_q{};


    std::array<double, 6> dq_max_{{2.0, 2.0, 2.0, 2.0, 2.5, 2.5}};
    std::array<double, 6> ddq_max_{{5, 5, 5, 5, 5, 5}};
};

}  // namespace franka_example_controllers