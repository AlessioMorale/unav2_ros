#ifndef UNAV2_HARDWARE_H
#define UNAV2_HARDWARE_H

#pragma once

#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <unav2_msgs/JointCommand.h>
#include <unav2_msgs/JointState.h>
namespace unav2_hardware {

class Unav2Hardware : public hardware_interface::RobotHW {
public:
  Unav2Hardware();
  void read();
  void write();

private:
  void joint_state_callback(const unav2_msgs::JointStateConstPtr &msg);

  ros::NodeHandle nh;
  ros::Subscriber jointstate_sub;
  realtime_tools::RealtimePublisher<unav2_msgs::JointCommand> jointcommand_pub;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  typedef struct joint {
    double position;
    double velocity;
    double effort;
    double command;
  } joint_t;

  std::vector<joint_t> joints;
  
  unav2_msgs::JointStateConstPtr jointstate_msg;
  std::mutex jointstate_msg_mutex;
};

} // namespace unav2_hardware
#endif /* UNAV2_HARDWARE_H */
