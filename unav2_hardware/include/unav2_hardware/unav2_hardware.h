#ifndef UNAV2_HARDWARE_H
#define UNAV2_HARDWARE_H

#pragma once

#include <mutex>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <unav2_msgs/JointCommand.h>
#include <unav2_msgs/JointState.h>
namespace unav2_hardware {

class Unav2Hardware : public hardware_interface::RobotHW {
public:
  Unav2Hardware(const ros::NodeHandle &nh);
  void update(const ros::TimerEvent &e);
  const std::string name = "Unav2Hardware";

private:
  void read(const ros::Time &, const ros::Duration &) override;
  void write(const ros::Time &, const ros::Duration &) override;

  void joint_state_callback(const unav2_msgs::JointStateConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber jointstate_sub_;
  realtime_tools::RealtimePublisher<unav2_msgs::JointCommand> jointcommand_pub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  typedef struct joint {
    double position;
    double velocity;
    double effort;
    double velocity_command;
  } joint_t;

  std::vector<joint_t> joints_;

  unav2_msgs::JointStateConstPtr jointstate_msg_;
  std::mutex jointstate_msg_mutex_;

  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double rate_;
  double v_error_;
};

} // namespace unav2_hardware
#endif /* UNAV2_HARDWARE_H */
